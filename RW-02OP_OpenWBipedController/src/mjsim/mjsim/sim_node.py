"""
MuJoCo Simulation ROS2 Node

Publishes:
  - /joint_states (sensor_msgs/JointState): position, velocity, effort
  - /imu/data (sensor_msgs/Imu): orientation, angular_velocity, linear_acceleration

Subscribes:
  - /mujoco_action (std_msgs/Float32MultiArray): MIT control commands
    Format: [pos0, kp0, kd0, pos1, kp1, kd1, ...] (3 values per joint)
    Or: [tau0, pos0, vel0, kp0, kd0, ...] (5 values per joint, full MIT mode)

Services:
  - /get_joint_limits (std_srvs/Trigger): Returns joint limits as JSON
    Response: {"names": [...], "lower": [...], "upper": [...]}

Usage:
  ros2 run mjsim sim_node --ros-args -p model:=panda
  ros2 run mjsim sim_node --ros-args -p model:=/path/to/model.xml
  
  # Query joint limits from another terminal:
  ros2 service call /get_joint_limits std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import json

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from mjsim.sim_manager import SimManager
from mjsim.cfgs import get_model_path, get_model_config
from builtin_interfaces.msg import Time


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')
        
        # 参数
        self.declare_parameter('model', 'panda')   # Model name or path
        self.declare_parameter('enable_viewer', True)
        self.declare_parameter('pub_rate', 100)    # Hz
        self.declare_parameter('sim_rate', 1.0)    # 加速比例
        self.declare_parameter('sync', False)      # 同步模式
        
        model = self.get_parameter('model').get_parameter_value().string_value
        enable_viewer = self.get_parameter('enable_viewer').get_parameter_value().bool_value
        pub_rate = self.get_parameter('pub_rate').get_parameter_value().integer_value
        sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        self.sync = self.get_parameter('sync').get_parameter_value().bool_value
        
        # Resolve model path
        try:
            xml_path = get_model_path(model)
        except ValueError as e:
            self.get_logger().error(f'\033[91m{e}\033[0m')
            raise
        
        # Initialize SimManager
        self.sim = SimManager(xml_path, enable_viewer=enable_viewer, viewer_sync_interval=30)
        if not self.sim.enable_camera_tracking("link_base", distance=2.0):
            self.sim.enable_camera_tracking("base_link", distance=2.0)
        
        # Apply default control parameters from model config
        config = get_model_config(model)
        if config is not None:
            self.sim.set_hold_position(
                kp=config.default_kp,
                kd=config.default_kd,
                pos_des=config.pd_stand_pos
            )
        else:
            # Fallback for direct path models
            self.sim.set_hold_position(kp=30.0, kd=1.0)
        
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Subscriber for control commands
        self.action_sub = self.create_subscription(
            Float32MultiArray,
            '/mujoco_action',
            self.action_callback,
            10
        )
        
        # Service to query joint limits
        self.joint_limits_srv = self.create_service(
            Trigger,
            '/get_joint_limits',
            self.joint_limits_callback
        )
        
        # Control data buffer
        self._action_data = None
        
        # Timer for simulation loop
        # sim_period = 1.0 / sim_rate
        sim_period = self.sim.model.opt.timestep / sim_rate
        self.sim_timer = self.create_timer(sim_period, self.sim_step)
        
        # Status tracking for _print_cmd_status_1s
        self._real_start_time = time.time()
        self._last_status_print_time = 0.0
        
        # Action callback tracking
        self._action_callback_count = 0
        self._last_action_time = time.time()
        self._max_action_gap = 0.0
        self.action_deal = True
        
        self._print_init_info(xml_path, pub_rate, sim_rate)
    
    def _print_init_info(self, xml_path: str, pub_rate: int, sim_rate: int):
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
        self.get_logger().info(f'\033[96mMujocoSimNode Started\033[0m')
        self.get_logger().info(f'\033[93mModel:\033[0m {xml_path}')
        self.get_logger().info(f'\033[93mActuators:\033[0m {self.sim.actuator_num}')
        self.get_logger().info(f'\033[93mSim rate:\033[0m {sim_rate} Hz')
        self.get_logger().info(f'\033[93mTopics:\033[0m')
        self.get_logger().info(f'  Pub: /joint_states, /imu/data')
        self.get_logger().info(f'  Sub: /mujoco_action')
        self.get_logger().info(f'  Srv: /get_joint_limits')
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
    
    def action_callback(self, msg: Float32MultiArray):
        """Handle incoming action commands"""
        # current_time = time.time()
        current_time = self.sim.sim_time
        
        # Track time gap since last action
        action_gap = current_time - self._last_action_time
        if action_gap > self._max_action_gap:
            self._max_action_gap = action_gap
        
        # Update tracking variables
        self._action_callback_count += 1
        self._last_action_time = current_time
        
        self._action_data = np.array(msg.data, dtype=np.float64)
        self.action_deal = False
    
    def joint_limits_callback(self, request, response):
        """
        Service callback to return joint limits information
        
        Response format (JSON in message field):
        {
            "names": ["joint1", "joint2", ...],
            "lower": [-1.0, -2.0, ...],
            "upper": [1.0, 2.0, ...]
        }
        """
        limits = self.sim.get_joint_limits()
        response.success = True
        response.message = json.dumps(limits)
        return response
    
    # MARK: Step
    def sim_step(self):
        """Main simulation step callback"""
        # Apply control if available
        if self._action_data is not None:
            n = self.sim.actuator_num
            data = self._action_data
            # print("收到消息, len = {n}")
            
            if len(data) == 3 * n:
                # Format: [pos0, kp0, kd0, pos1, kp1, kd1, ...]
                # tau_ff = 0, vel_des = 0
                tau_ff = np.zeros(n)
                pos_des = np.array([data[i * 3] for i in range(n)])
                vel_des = np.zeros(n)
                kp = np.array([data[i * 3 + 1] for i in range(n)])
                kd = np.array([data[i * 3 + 2] for i in range(n)])
                self.sim.set_mit_commands(tau_ff, pos_des, vel_des, kp, kd)
                
            elif len(data) == 5 * n:
                # Full MIT format: [tau0, pos0, vel0, kp0, kd0, ...]
                tau_ff = np.array([data[i * 5] for i in range(n)])
                pos_des = np.array([data[i * 5 + 1] for i in range(n)])
                vel_des = np.array([data[i * 5 + 2] for i in range(n)])
                kp = np.array([data[i * 5 + 3] for i in range(n)])
                kd = np.array([data[i * 5 + 4] for i in range(n)])
                self.sim.set_mit_commands(tau_ff, pos_des, vel_des, kp, kd)
                
            elif len(data) == n:
                # Pure torque mod
                self.sim.set_torque(data)
        
        if self.sync:
            """同步模式"""
            if not self.action_deal:
                self.sim.step()
                self.action_deal = True
        else:
            self.sim.step()
        
        self._publish_state()
        
        # Check viewer
        if not self.sim.is_running():
            self.get_logger().info('Viewer closed, shutting down...')
            rclpy.shutdown()
        
        # Print status every 1s
        self._print_cmd_status_1s()
    
    def _print_cmd_status_1s(self):
        """仿真统计状态：1s"""
        sim_time = self.sim.sim_time
        
        # Only print every 1 second of simulation time
        if sim_time - self._last_status_print_time >= 1.0:
            real_elapsed = time.time() - self._real_start_time
            sim_real_ratio = sim_time / real_elapsed if real_elapsed > 0 else 0.0
            
            # Calculate max action gap in milliseconds
            max_gap_ms = self._max_action_gap * 1000.0
            
            print(f'\033[94m[Status: {sim_time:.2f}s]\033[0m, ',
                # f'sim/real: {sim_real_ratio:.2f}x, ',
                f'{sim_real_ratio:.2f}x, ',
                f'action_cb: {self._action_callback_count}, ',
                f'max_gap: {max_gap_ms:.1f}ms'
            )
            
            # Reset tracking counters
            self._action_callback_count = 0
            self._max_action_gap = 0.0
            self._last_status_print_time = sim_time
    
    def _publish_state(self):
        """Publish joint state and IMU data"""
        # Use ROS node time for header
        # now = self.get_clock().now().to_msg()
        now = self.get_simt_msg()
        
        
        # JointState
        js = self.sim.get_joint_state()
        js.header.stamp = now
        self.joint_state_pub.publish(js)
        
        # IMU
        imu = self.sim.get_imu_data()
        imu.header.stamp = now
        self.imu_pub.publish(imu)

    def get_simt_msg(self, ):
        # 仿真时间获取
        t_sim = self.sim.sim_time
        time_msg = Time()
        time_msg.sec = int(t_sim)
        time_msg.nanosec = int((t_sim - int(t_sim)) * 1e9)
        return time_msg


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MujocoSimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'\033[91mError: {e}\033[0m')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
