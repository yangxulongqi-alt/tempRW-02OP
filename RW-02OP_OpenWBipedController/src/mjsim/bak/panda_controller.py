"""
Panda Robot Controller Node

Subscribes:
  - /joint_states (sensor_msgs/JointState): position, velocity, effort
  - /imu/data (sensor_msgs/Imu): orientation, angular_velocity, linear_acceleration

Publishes:
  - /mujoco_action (std_msgs/Float32MultiArray): MIT control commands
    Format: [pos0, kp0, kd0, pos1, kp1, kd1, ...] (3 values per joint)

Usage:
  ros2 run mjsim panda_controller
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

from mjsim.policy.policy import Policy
from mjsim.policy.policyCfg import load_policy_config
from mjsim.cfgs import POLICY_REGISTRY, MJCF_MODEL_REGISTRY


class PandaController(Node):
    def __init__(self):
        super().__init__('panda_controller')
        
        # ROS参数
        self.declare_parameter('control_rate', 50)  # Hz
        self.declare_parameter('policy_name', 'panda_walk')
        self.declare_parameter('device', 'cuda')
        
        control_rate = self.get_parameter('control_rate').get_parameter_value().integer_value
        policy_name = self.get_parameter('policy_name').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        
        # 策略配置文件
        self.policy_cfg = load_policy_config(policy_name)
        self.num_joints = self.policy_cfg.num_joints
        
        # 策略模型
        policy_path = POLICY_REGISTRY[policy_name] / 'policy.pt'
        obs_specs = self.policy_cfg.get_obs_specs()  # Returns (name, dim, scale) tuples
        
        self.policy = Policy(
            model_path=str(policy_path),
            device=device,
            obs_specs=obs_specs,
            history_length=self.policy_cfg.obs_history_length
        )
        self.policy.warm_up()
        
        # 状态缓冲区
        self.joint_state = None
        self.imu_data = None
        self.last_action = np.zeros(self.num_joints)
        
        # Motor name mapping: policy_cfg.joint_names -> model_cfg.motor_names (xml order)
        model_cfg = MJCF_MODEL_REGISTRY.get('panda')
        self.xml_motor_names = model_cfg.motor_names if model_cfg else []
        self.motor_dir = np.array(model_cfg.motor_dir) if model_cfg and model_cfg.motor_dir else np.ones(self.num_joints)
        self.policy_joint_names = self.policy_cfg.joint_names
        self.policy_to_xml_idx = self._build_policy_to_xml_mapping()
        
        # 控制参数 (convert from policy order to xml order)
        self.default_kp = self._reorder_policy_to_xml(self.policy_cfg.stiffness)
        self.default_kd = self._reorder_policy_to_xml(self.policy_cfg.damping)
        self.default_pos = self._reorder_policy_to_xml(self.policy_cfg.default_pos)
        self.action_scale = self._reorder_policy_to_xml(self.policy_cfg.action_scale)
        self.action_offset = self._reorder_policy_to_xml(self.policy_cfg.action_offset)
        
        self._print_kp_kd_pos()
        
        # 消息处理
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states',
            self.joint_state_callback, 10 )
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.action_pub = self.create_publisher(Float32MultiArray, '/mujoco_action', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Control timer
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)
        self.loop_cnt = 0
        self.loop_start_time = 0.0
        self._last_status_print_time = 0.0
        
        self._print_init_info(control_rate)
        self._print_obs_specs(obs_specs)
    
    # MARK: Motor Mapping
    def _build_policy_to_xml_mapping(self) -> np.ndarray:
        """
        Build index mapping from policy joint order to xml motor order.
        
        policy_cfg.joint_names: joint order defined in model_info.json
        xml_motor_names: motor order defined in xml/ModelConfig
        
        Returns:
            policy_to_xml_idx: array where policy_to_xml_idx[policy_i] = xml_i
        """
        n = len(self.policy_joint_names)
        policy_to_xml_idx = np.zeros(n, dtype=int)
        
        self.get_logger().info(f'\033[92mBuilding policy->xml mapping:\033[0m')
        for policy_i, name in enumerate(self.policy_joint_names):
            if name in self.xml_motor_names:
                xml_i = self.xml_motor_names.index(name)
                policy_to_xml_idx[policy_i] = xml_i
                self.get_logger().info(f'  policy[{policy_i:2d}] {name:30s} -> xml[{xml_i:2d}]')
            else:
                self.get_logger().warn(f'  policy[{policy_i:2d}] {name:30s} NOT FOUND in xml!')
        
        return policy_to_xml_idx
    
    def _reorder_policy_to_xml(self, policy_arr: np.ndarray) -> np.ndarray:
        """Reorder array from policy order to xml order"""
        result = np.zeros_like(policy_arr)
        for policy_i in range(len(self.policy_to_xml_idx)):
            xml_i = self.policy_to_xml_idx[policy_i]
            result[xml_i] = policy_arr[policy_i]
        return result
    
    def _reorder_xml_to_policy(self, xml_arr: np.ndarray) -> np.ndarray:
        """Reorder array from xml order to policy order"""
        result = np.zeros_like(xml_arr)
        for policy_i in range(len(self.policy_to_xml_idx)):
            xml_i = self.policy_to_xml_idx[policy_i]
            result[policy_i] = xml_arr[xml_i]
        return result
    
    # MARK: Info
    def _print_kp_kd_pos(self):
        """Print control parameters with colored output"""
        np.set_printoptions(precision=3, suppress=True)
        self.get_logger().info(f'\033[93mKp:\033[0m {self.default_kp}')
        self.get_logger().info(f'\033[93mKd:\033[0m {self.default_kd}')
        self.get_logger().info(f'\033[93mDefault Pos:\033[0m {self.default_pos}')
    
    def _print_obs_specs(self, obs_specs: list):
        """Print observation specs with colored output"""
        self.get_logger().info(f'\033[96m{"="*60}\033[0m')
        self.get_logger().info(f'\033[96mObservation Specs\033[0m')
        for spec in obs_specs:
            if len(spec) == 3:
                name, dim, scale = spec
                self.get_logger().info(f'  \033[93m{name:20}\033[0m dim={dim:3d} scale={scale:.6f}')
            else:
                self.get_logger().info(f'  \033[93m{str(spec):50}\033[0m')
        self.get_logger().info(f'\033[96m{"="*60}\033[0m')

    def _print_init_info(self, control_rate: int):
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
        self.get_logger().info(f'\033[96mPandaController Started\033[0m')
        self.get_logger().info(f'\033[93mPolicy:\033[0m {self.policy_cfg.policy_path.name}')
        self.get_logger().info(f'\033[93mNum joints:\033[0m {self.num_joints}')
        self.get_logger().info(f'\033[93mControl rate:\033[0m {control_rate} Hz')
        self.get_logger().info(f'\033[93mTopics:\033[0m')
        self.get_logger().info(f'  Sub: /joint_states, /imu/data')
        self.get_logger().info(f'  Pub: /mujoco_action')
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
    
    def _print_cmd_status_1s(self):
        """命令行窗口，每1s显示数据"""
        elapsed_time = time.time() - self.loop_start_time
        
        if self.loop_cnt%50==0:
            print(f'\033[94m[Status]\033[0m: {elapsed_time:.2f}s',
            f"loop: {self.loop_cnt}",
            )
            self._last_status_print_time = elapsed_time

    
    def _build_xml_to_policy_mapping(self, msg_names: list):
        """Build mapping from xml/msg joint order to policy order"""
        n = len(self.policy_joint_names)
        self.xml_to_policy_idx = np.zeros(n, dtype=int)
        
        self.get_logger().info(f'\033[92mBuilding xml->policy mapping (for q/dq):\033[0m')
        for policy_i, name in enumerate(self.policy_joint_names):
            if name in msg_names:
                xml_i = msg_names.index(name)
                self.xml_to_policy_idx[policy_i] = xml_i
                self.get_logger().info(f'  xml[{xml_i:2d}] {name:30s} -> policy[{policy_i:2d}]')
            else:
                self.get_logger().warn(f'  {name} NOT FOUND in msg!')
    
    def imu_callback(self, msg: Imu):
        """Store latest IMU data"""
        self.imu_data = msg
    
    # MARK: Loop
    def control_loop(self):
        if(self.loop_cnt == 0):
            self.loop_start_time = time.time()
        self.loop_cnt = self.loop_cnt + 1
        self._print_cmd_status_1s()

        """Main control loop with policy inference"""
        if self.joint_state is None or self.imu_data is None:
            return
        if not hasattr(self, 'q_policy') or self.q_policy is None:
            return
        
        # Get IMU data
        quat = self.imu_data.orientation
        ang_vel = self.imu_data.angular_velocity
        proj_gravity = self._quat_to_proj_gravity(quat)
        
        # Set policy observations
        obs_base_ang_vel = np.array([ang_vel.x, ang_vel.y, ang_vel.z])
        obs_proj_gravity = proj_gravity
        # obs_vel_cmd = np.zeros(3)
        obs_vel_cmd = np.array([0.5, 0.0, 0.0])
        obs_joint_pos = self.q_policy - self.policy_cfg.default_pos
        obs_joint_vel = self.dq_policy
        obs_last_action = self.last_action
        obs_gait_phase = self.get_realtime_phase_numpy(time.time(), 0.6)
        
        self.policy.set_observation("base_ang_vel", obs_base_ang_vel)
        self.policy.set_observation("projected_gravity", obs_proj_gravity)
        self.policy.set_observation("velocity_commands", obs_vel_cmd)
        self.policy.set_observation("joint_pos_rel", obs_joint_pos)
        self.policy.set_observation("joint_vel_rel", obs_joint_vel)
        self.policy.set_observation("last_action", obs_last_action)
        self.policy.set_observation("gait_phase", obs_gait_phase)
        
        # 运行推理
        raw_action = self.policy.step()
        self.last_action = raw_action.copy()
        
        # Scale action to target position (policy order)
        target_pos_policy = raw_action * self.policy_cfg.action_scale + self.policy_cfg.action_offset
        
        # Convert to xml order for control, apply motor_dir
        target_pos_xml = self._reorder_policy_to_xml(target_pos_policy) * self.motor_dir
        
        # Build MIT control command (xml order)
        action = self.compute_action(target_pos_xml)
        
        # Debug output (uncomment to enable)
        # self._print_debug(obs_base_ang_vel, obs_proj_gravity, obs_vel_cmd, 
        #                   obs_joint_pos, obs_joint_vel, obs_last_action, 
        #                   obs_gait_phase, raw_action, target_pos_xml)
        
        # Publish action
        self.publish_action(action)

    def joy_callback(self, msg: Joy):
        """手柄消息"""


    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state and convert to policy order"""
        # Build xml->policy mapping on first message
        if not hasattr(self, 'xml_to_policy_idx') or self.xml_to_policy_idx is None:
            self._build_xml_to_policy_mapping(list(msg.name))
        
        # Extract q/dq from msg (xml order), apply motor_dir, and convert to policy order
        q_xml = np.array(msg.position) * self.motor_dir
        dq_xml = np.array(msg.velocity) * self.motor_dir
        self.q_policy = self._reorder_xml_to_policy(q_xml)
        self.dq_policy = self._reorder_xml_to_policy(dq_xml)
        self.joint_state = msg
    
    def _print_debug(self, base_ang_vel, proj_gravity, vel_cmd, joint_pos, 
                     joint_vel, last_action, gait_phase, raw_action, target_pos):
        """Print current observations and control for debugging"""
        np.set_printoptions(precision=3, suppress=True)
        self.get_logger().info(f'\033[96m{"="*60}\033[0m')
        self.get_logger().info(f'\033[93mObservations:\033[0m')
        self.get_logger().info(f'  base_ang_vel     : {base_ang_vel}')
        self.get_logger().info(f'  proj_gravity     : {proj_gravity}')
        self.get_logger().info(f'  velocity_commands: {vel_cmd}')
        self.get_logger().info(f'  joint_pos_rel    : {joint_pos[:5]}...')
        self.get_logger().info(f'  joint_vel_rel    : {joint_vel[:5]}...')
        self.get_logger().info(f'  last_action      : {last_action[:5]}...')
        self.get_logger().info(f'  gait_phase       : {gait_phase}')
        self.get_logger().info(f'\033[93mControl:\033[0m')
        self.get_logger().info(f'  raw_action       : {raw_action[:5]}...')
        self.get_logger().info(f'  target_pos       : {target_pos[:5]}...')
        self.get_logger().info(f'\033[96m{"="*60}\033[0m')
    
    def get_realtime_phase_numpy(self, current_time: float, period: float) -> np.ndarray:
        """
        基于绝对时间计算步态相位 (NumPy版)

        Returns:
            np.ndarray: 形状为 (2,) 的数组，包含 [sin(phi), cos(phi)]
        """
        phi = (current_time % period) / period
        angle = phi * 2 * np.pi
        
        return np.array([np.sin(angle), np.cos(angle)], dtype=np.float32)

    
    def _quat_to_proj_gravity(self, quat) -> np.ndarray:
        """Convert quaternion to projected gravity vector"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        # Rotate [0, 0, -1] by inverse quaternion
        gx = 2.0 * (x * z - w * y)
        gy = 2.0 * (y * z + w * x)
        gz = w * w - x * x - y * y + z * z
        return np.array([-gx, -gy, -gz])
    
    # MARK: 计算输出
    def compute_action(self, target_pos: np.ndarray) -> np.ndarray:
        """
        Compute MIT control action from target positions.
        
        Args:
            target_pos: Target joint positions [num_joints]
        
        Returns:
            action: Control command in format [pos0, kp0, kd0, pos1, kp1, kd1, ...]
        """
        n = min(len(target_pos), self.num_joints)
        action = np.zeros(3 * self.num_joints)
        
        for i in range(n):
            action[i * 3] = target_pos[i]         # pos_des
            action[i * 3 + 1] = self.default_kp[i]  # kp
            action[i * 3 + 2] = self.default_kd[i]  # kd
        
        return action
    
    def publish_action(self, action: np.ndarray):
        """Publish control action to /mujoco_action"""
        msg = Float32MultiArray()
        msg.data = action.astype(np.float32).tolist()
        self.action_pub.publish(msg)



# MARK: Main
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PandaController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'\033[91mError: {e}\033[0m')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
