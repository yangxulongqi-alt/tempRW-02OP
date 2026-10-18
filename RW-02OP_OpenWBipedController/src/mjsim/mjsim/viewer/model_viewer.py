"""
Model Viewer ROS2 Node

用于可视化机器人状态：订阅JointState和IMU消息，在MuJoCo中显示

Subscribes:
  - /joint_states (sensor_msgs/JointState): 关节位置、速度、力矩
  - /imu/data (sensor_msgs/Imu): 姿态四元数、角速度、线加速度

Usage:
  ros2 run mjsim model_viewer --ros-args -p model:=panda
  ros2 run mjsim model_viewer --ros-args -p model:=/path/to/model.xml
"""

import rclpy
from rclpy.node import Node
import numpy as np
import mujoco
import mujoco.viewer as viewer
import time
import os
import sys

from sensor_msgs.msg import JointState, Imu
from scipy.spatial.transform import Rotation as R

from mjsim.cfgs import get_model_path, get_model_config


class ModelViewerNode(Node):
    """
    MuJoCo Model Viewer Node
    
    订阅JointState和IMU消息，实时可视化机器人状态
    """
    
    def __init__(self):
        super().__init__('model_viewer_node')
        
        # Parameters
        self.declare_parameter('model', 'panda')
        self.declare_parameter('update_rate', 60)  # Hz
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter('imu_topic', '/imu/data')
        
        model_name = self.get_parameter('model').get_parameter_value().string_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        joint_topic = self.get_parameter('joint_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
        # Resolve model path and get config
        try:
            xml_path = get_model_path(model_name)
        except ValueError as e:
            self.get_logger().error(f'\033[91m{e}\033[0m')
            raise
        
        # Get model config for motor_dir
        self._model_config = get_model_config(model_name)
        # self._motor_dir_map = {}
        # if self._model_config is not None and self._model_config.motor_names and self._model_config.motor_dir:
        #     for name, direction in zip(self._model_config.motor_names, self._model_config.motor_dir):
        #         self._motor_dir_map[name] = direction
        #     self.get_logger().info(f'\033[93mMotor direction mapping loaded: {len(self._motor_dir_map)} joints\033[0m')
        
        # Load MuJoCo model
        if not os.path.exists(xml_path):
            self.get_logger().error(f'\033[91mModel file not found: {xml_path}\033[0m')
            sys.exit(1)
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # Build actuator name -> joint qpos/dof address mapping
        self.actuator_num = self.model.nu
        self._actuator_name_to_idx = {}
        self._actuator_jnt_qposadr = np.zeros(self.actuator_num, dtype=np.int32)
        self._actuator_jnt_dofadr = np.zeros(self.actuator_num, dtype=np.int32)
        
        for i in range(self.actuator_num):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator_{i}"
            self._actuator_name_to_idx[name] = i
            trnid = self.model.actuator_trnid[i, 0]
            self._actuator_jnt_qposadr[i] = self.model.jnt_qposadr[trnid]
            self._actuator_jnt_dofadr[i] = self.model.jnt_dofadr[trnid]
        
        # Find floating base (if exists)
        self._has_floating_base = False
        self._base_qposadr = 0
        if self.model.nq > self.actuator_num:
            # Check if first joint is freejoint (7 DoF: 3 pos + 4 quat)
            if self.model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
                self._has_floating_base = True
                self._base_qposadr = self.model.jnt_qposadr[0]
                self.get_logger().info('\033[93mFloating base detected\033[0m')
        
        # Launch passive viewer
        self.viewer = viewer.launch_passive(
            self.model, self.data,
            show_left_ui=False, show_right_ui=False
        )
        self.viewer.cam.distance = 3.0
        self.viewer.cam.elevation = -20
        self.viewer.cam.azimuth = 135
        
        # State buffers
        self._joint_state = None
        self._imu_data = None
        self._last_joint_time = 0.0
        self._last_imu_time = 0.0
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            joint_topic,
            self.joint_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        
        # Update timer
        update_period = 1.0 / update_rate
        self.update_timer = self.create_timer(update_period, self.update_viewer)
        
        # Status timer
        self._start_time = time.time()
        self._last_status_time = 0.0
        
        self._print_init_info(xml_path, update_rate, joint_topic, imu_topic)
    
    def _print_init_info(self, xml_path: str, update_rate: int, joint_topic: str, imu_topic: str):
        """Print initialization info"""
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
        self.get_logger().info(f'\033[96mModelViewerNode Started\033[0m')
        self.get_logger().info(f'\033[93mModel:\033[0m {xml_path}')
        self.get_logger().info(f'\033[93mActuators:\033[0m {self.actuator_num}')
        self.get_logger().info(f'\033[93mFloating Base:\033[0m {self._has_floating_base}')
        self.get_logger().info(f'\033[93mUpdate Rate:\033[0m {update_rate} Hz')
        self.get_logger().info(f'\033[93mSubscribing:\033[0m')
        self.get_logger().info(f'  - {joint_topic} (JointState)')
        self.get_logger().info(f'  - {imu_topic} (Imu)')
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
    
    def joint_callback(self, msg: JointState):
        """Handle incoming joint state"""
        self._joint_state = msg
        self._last_joint_time = time.time()
    
    def imu_callback(self, msg: Imu):
        """Handle incoming IMU data"""
        self._imu_data = msg
        self._last_imu_time = time.time()
    
    def update_viewer(self):
        """Update MuJoCo model state and sync viewer"""
        
        # Check if viewer is still running
        if not self.viewer.is_running():
            self.get_logger().info('Viewer closed, shutting down...')
            rclpy.shutdown()
            return
        
        # Apply joint state
        if self._joint_state is not None:
            self._apply_joint_state(self._joint_state)
        
        # Apply IMU orientation (for floating base)
        if self._imu_data is not None and self._has_floating_base:
            self._apply_imu_orientation(self._imu_data)
        
        # Forward kinematics (update visual positions)
        mujoco.mj_forward(self.model, self.data)
        
        # Sync viewer
        self.viewer.sync()
        
        # Print status every 2 seconds
        self._print_status()
    
    def _apply_joint_state(self, msg: JointState):
        """Apply joint positions from JointState message"""
        for i, name in enumerate(msg.name):
            if name in self._actuator_name_to_idx:
                idx = self._actuator_name_to_idx[name]
                qpos_adr = self._actuator_jnt_qposadr[idx]
                dof_adr = self._actuator_jnt_dofadr[idx]
                
                # Get motor direction (default 1 if not specified)
                # motor_dir = self._motor_dir_map.get(name, 1)
                
                # Set position (apply motor_dir sign)
                if i < len(msg.position):
                    self.data.qpos[qpos_adr] = msg.position[i] # * motor_dir
                
                # Set velocity (apply motor_dir sign)
                if i < len(msg.velocity):
                    self.data.qvel[dof_adr] = msg.velocity[i] # * motor_dir
    
    def _apply_imu_orientation(self, msg: Imu):
        """Apply IMU orientation to floating base"""
        if not self._has_floating_base:
            return
        
        # Get quaternion from IMU (ROS format: x, y, z, w)
        quat_ros = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Convert to MuJoCo format: w, x, y, z
        quat_mujoco = np.array([
            quat_ros[3],  # w
            quat_ros[0],  # x
            quat_ros[1],  # y
            quat_ros[2]   # z
        ])
        
        # Apply to base qpos (position at 0:3, quaternion at 3:7)
        qpos_adr = self._base_qposadr
        self.data.qpos[qpos_adr + 3:qpos_adr + 7] = quat_mujoco
    
    def _print_status(self):
        """Print status every 2 seconds"""
        now = time.time()
        if now - self._last_status_time < 2.0:
            return
        self._last_status_time = now
        
        elapsed = now - self._start_time
        
        # Data freshness
        joint_age = now - self._last_joint_time if self._last_joint_time > 0 else float('inf')
        imu_age = now - self._last_imu_time if self._last_imu_time > 0 else float('inf')
        
        joint_status = f'{joint_age:.1f}s ago' if joint_age < 100 else 'No data'
        imu_status = f'{imu_age:.1f}s ago' if imu_age < 100 else 'No data'
        
        print(f'\033[94m[Status]\033[0m elapsed: {elapsed:.1f}s, '
              f'joint: {joint_status}, imu: {imu_status}')
    
    def destroy_node(self):
        """Clean up resources"""
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ModelViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'\033[91mError: {e}\033[0m')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
