"""
Controller Base Class

An abstract base class for robot controllers that provides:
  - ROS2 node infrastructure (parameters, subscribers, publishers, timer)
  - State management (joint states, IMU data)
  - Control loop skeleton
  - Action publishing

Subclasses should implement:
  - get_node_name(): Return the ROS node name
  - get_default_parameters(): Return default parameter values
  - setup_observations(): Set observations before policy inference
  - on_init(): Optional post-initialization hook

Subscribes:
  - /joint_states (sensor_msgs/JointState): position, velocity, effort
  - /imu/data (sensor_msgs/Imu): orientation, angular_velocity, linear_acceleration
  - joy (sensor_msgs/Joy): joystick axes and buttons

Publishes:
  - /mujoco_action (std_msgs/Float32MultiArray): MIT control commands
    Format: [pos0, kp0, kd0, pos1, kp1, kd1, ...] (3 values per joint)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from dataclasses import dataclass

from sensor_msgs.msg import JointState, Imu, Joy
from std_msgs.msg import Float32MultiArray

from mjsim.policy.policy_controller import PolicyController

# from cmotor_interface.msg import Cmotorcommand   # Shiro Extra


@dataclass
class ControllerParams:
    """Controller parameters"""
    control_rate: int = 50
    policy_name: str = 'panda_walk'
    model_name: str = 'panda'
    device: str = 'cuda'
    auto_init_controller: bool = True  # Whether to auto-initialize PolicyController
    joint_state_timeout: float = 2.0  # Timeout in seconds for joint_states during inference


class ControllerBase(Node, ABC):
    """
    Abstract base class for policy-based robot controllers.
    
    Provides common infrastructure for:
      - ROS2 node setup
      - State management
      - Control loop execution
      - Action publishing
    
    Subclasses must implement:
      - get_node_name(): Return the ROS node name
      - get_default_parameters(): Return default ControllerParams
      - setup_observations(): Set observations before policy inference
    """
    
    def __init__(self):
        # Get node name from subclass
        node_name = self.get_node_name()
        super().__init__(node_name)
        
        # Get default parameters from subclass
        defaults = self.get_default_parameters()
        
        # Declare ROS parameters
        self.declare_parameter('control_rate', defaults.control_rate)
        self.declare_parameter('policy_name', defaults.policy_name)
        self.declare_parameter('model_name', defaults.model_name)
        self.declare_parameter('device', defaults.device)
        
        # Get parameter values
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().integer_value
        self.policy_name = self.get_parameter('policy_name').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        
        # Initialize PolicyController (optional, can be set externally)
        self.controller: Optional[PolicyController] = None
        if defaults.auto_init_controller:
            self.controller = self._create_default_controller()
        
        # State buffers
        self.joint_state: Optional[JointState] = None
        self.imu_data: Optional[Imu] = None
        self.joy_data: Optional[Joy] = None
        self.q_policy: Optional[np.ndarray] = None
        self.dq_policy: Optional[np.ndarray] = None
        
        # Mapping built flag
        self._mapping_built = False
        
        # ROS subscribers and publishers
        self._setup_ros_interfaces()
        # self.motor_pub = self.create_publisher(
        #     Cmotorcommand, 
        #     '/motor_commands', 
        #     10
        # )
        
        # Control timer
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self._control_loop_wrapper)
        
        # Loop state
        self.loop_cnt = 0
        self.loop_start_time = 0.0
        
        # Safety: joint_states timeout monitoring
        self._inference_started = False  # True after first successful inference
        self._last_joint_state_time: Optional[float] = None  # Last time joint_states received
        self._joint_state_timeout = defaults.joint_state_timeout  # Timeout threshold
        
        # Call subclass post-init hook
        self.on_init()
        
        # Print initialization info
        self._print_init_info()
    
    # ========== Controller Management ==========
    # MARK: 多策略管理
    
    def _create_default_controller(self) -> PolicyController:
        """Create the default PolicyController."""
        return PolicyController(
            policy_name=self.policy_name,
            model_name=self.model_name,
            device=self.device,
            logger=self.get_logger().info
        )
    
    def set_controller(self, controller: PolicyController):
        """
        Set or switch the PolicyController.
        
        Args:
            controller: A PolicyController instance
        
        Example:
            new_controller = PolicyController('new_policy', 'new_model')
            self.set_controller(new_controller)
        """
        self.controller = controller
        self._mapping_built = False  # Reset mapping for new controller
        # Reset q_policy and dq_policy to force re-computation with new controller
        self.q_policy = None
        self.dq_policy = None
        self.get_logger().info(f'\033[92mSwitched to controller: policy={controller.policy_name}, model={controller.model_name}\033[0m')
    
    def get_controller(self) -> Optional[PolicyController]:
        """Get the current PolicyController."""
        return self.controller
    
    # ========== Abstract Methods (must be implemented by subclass) ==========
    # MARK: 接口方法
    
    @abstractmethod
    def get_node_name(self) -> str:
        """
        Return the ROS node name.
        
        Example:
            return 'panda_controller'
        """
        pass
    
    @abstractmethod
    def get_default_parameters(self) -> ControllerParams:
        """
        Return default controller parameters.
        
        Example:
            return ControllerParams(
                control_rate=50,
                policy_name='panda_walk',
                model_name='panda',
                device='cuda'
            )
        """
        pass
    
    @abstractmethod
    def setup_observations(self) -> bool:
        """
        Set up observations for policy inference.
        
        Called each control loop before compute_action().
        Use self.controller.set_observation() to set observations.
        
        Available state:
            - self.q_policy: Joint positions in policy order
            - self.dq_policy: Joint velocities in policy order
            - self.imu_data: Latest IMU message
            - self.joint_state: Latest JointState message
            - self.controller: PolicyController instance
        
        Returns:
            True if observations are ready, False to skip this control cycle
        
        Example:
            quat = self.imu_data.orientation
            ang_vel = self.imu_data.angular_velocity
            proj_gravity = self.controller._quat_to_proj_gravity(quat)
            
            self.controller.set_observation("base_ang_vel", 
                np.array([ang_vel.x, ang_vel.y, ang_vel.z]))
            self.controller.set_observation("projected_gravity", proj_gravity)
            # ... more observations
            
            return True
        """
        pass
    
    # ========== Optional Hooks (can be overridden) ==========
    
    def on_init(self):
        """
        Post-initialization hook.
        Called after base class initialization is complete.
        Override to add custom initialization logic.
        """
        pass
    
    def on_control_loop_start(self):
        """
        Called at the start of each control loop iteration.
        Override to add custom logic before observation setup.
        """
        pass
    
    def on_action_computed(self, action: np.ndarray) -> np.ndarray:
        """
        Called after action is computed, before publishing.
        Override to modify or log the action.
        
        Args:
            action: The computed MIT action array
        
        Returns:
            The (possibly modified) action to publish
        """
        return action

    def on_action_published(self):
        """
        Action发布之后的回调函数
        """
        pass
    
    def get_topic_joint_states(self) -> str:
        """Return the joint states topic name. Override to customize."""
        return '/joint_states'
    
    def get_topic_imu(self) -> str:
        """Return the IMU topic name. Override to customize."""
        return '/imu/data'
    
    def get_topic_action(self) -> str:
        """Return the action topic name. Override to customize."""
        return '/mujoco_action'
    
    def get_topic_joy(self) -> str:
        """Return the joy topic name. Override to customize."""
        return 'joy'
    
    def get_status_print_interval(self) -> int:
        """Return status print interval in loop counts. Override to customize."""
        return 50  # Print every 50 loops (1s at 50Hz)
    
    def get_joint_state_timeout(self) -> float:
        """Return joint_states timeout in seconds. Override to customize."""
        return self._joint_state_timeout
    
    # ========== ROS Setup ==========
    # MARK: ROS
    
    def _setup_ros_interfaces(self):
        """Set up ROS subscribers and publishers."""
        self.joint_state_sub = self.create_subscription(
            JointState, 
            self.get_topic_joint_states(), 
            self._joint_state_callback, 
            10
        )
        self.imu_sub = self.create_subscription(
            Imu, 
            self.get_topic_imu(), 
            self._imu_callback, 
            10
        )
        self.joy_sub = self.create_subscription(
            Joy,
            self.get_topic_joy(),
            self._joy_callback,
            10
        )
        self.action_pub = self.create_publisher(
            Float32MultiArray, 
            self.get_topic_action(), 
            10
        )
    
    # ========== Callbacks ==========
    # MARK: 回调处理
    
    def _imu_callback(self, msg: Imu):
        """Store latest IMU data."""
        self.imu_data = msg
    
    def _joy_callback(self, msg: Joy):
        """Store latest joy data."""
        self.joy_data = msg
    
    def _joint_state_callback(self, msg: JointState):
        """Store latest joint state and convert to policy order."""
        # Store raw message
        self.joint_state = msg
        
        # Update last received time for timeout monitoring
        self._last_joint_state_time = time.time()
        
        # Skip conversion if controller not set
        if self.controller is None:
            return
        
        # Build xml->policy mapping on first message
        if not self._mapping_built:
            self.controller.build_xml_to_policy_mapping(list(msg.name))
            self._mapping_built = True
        
        # Convert q/dq from XML to policy order with motor direction
        q_xml = np.array(msg.position)
        dq_xml = np.array(msg.velocity)
        
        q_with_dir = q_xml * self.controller.motor_dir
        dq_with_dir = dq_xml * self.controller.motor_dir
        
        self.q_policy = self.controller._reorder_xml_to_policy(q_with_dir)
        self.dq_policy = self.controller._reorder_xml_to_policy(dq_with_dir)
    
    # ========== Control Loop ==========
    # MARK: Loop
    def _control_loop_wrapper(self):
        """Main control loop wrapper with timing and status."""
        # Initialize timing on first loop
        if self.loop_cnt == 0:
            self.loop_start_time = time.time()
        self.loop_cnt += 1
        
        # Print status periodically
        self._print_status()
        
        # Call subclass hook
        self.on_control_loop_start()
        
        # Wait for data
        if not self._is_data_ready():
            return
        
        # Mark inference as started on first successful data ready
        if not self._inference_started:
            self._inference_started = True
            self.get_logger().info('\033[92m[Safety] Inference started, joint_states timeout monitoring enabled\033[0m')
        
        # Check for joint_states timeout during inference
        if self._check_joint_state_timeout():
            return  # Node is shutting down
        
        # Set up observations (subclass implementation)
        if not self.setup_observations():
            return
        
        # Compute action
        action = self.controller.compute_action()
        
        # Allow subclass to modify action
        action = self.on_action_computed(action)
        
        # Publish action
        self._publish_action(action)

        self.on_action_published()
    
    def _is_data_ready(self) -> bool:
        """Check if all required data is available."""
        if self.controller is None:
            return False
        if self.joint_state is None:
            return False
        if self.imu_data is None:
            return False
        if self.q_policy is None:
            return False
        return True
    
    def _check_joint_state_timeout(self) -> bool:
        """
        Check if joint_states has timed out during inference.
        
        Returns:
            True if timeout occurred and node is shutting down, False otherwise
        """
        if not self._inference_started:
            return False
        
        if self._last_joint_state_time is None:
            return False
        
        elapsed_since_last = time.time() - self._last_joint_state_time
        
        if elapsed_since_last > self._joint_state_timeout:
            self.get_logger().error(
                f'\033[91m[Safety] CRITICAL: No joint_states received for {elapsed_since_last:.2f}s '
                f'(timeout: {self._joint_state_timeout}s). Shutting down node for safety!\033[0m'
            )
            self.get_logger().error(
                f'\033[91m[Safety] Please check if the joint_states publisher is still running.\033[0m'
            )
            # Cancel the control timer to stop further processing
            self.control_timer.cancel()
            # Request shutdown
            raise SystemExit(f'Joint states timeout: no data received for {elapsed_since_last:.2f}s')
        
        return False
    
    def _publish_action(self, action: np.ndarray):
        """Publish control action to action topic."""
        msg = Float32MultiArray()
        msg.data = action.astype(np.float32).tolist()
        self.action_pub.publish(msg)
        # self.action_pub.publish(msg)
        # Shiro Extra
        # msg = Cmotorcommand()
        # msg.motor_ids = ["joint_left_shoulder_pitch","joint_left_shoulder_roll","joint_left_shoulder_yaw", "joint_left_elbow","joint_left_wrist",
        #              "joint_right_shoulder_pitch","joint_right_shoulder_roll", "joint_right_shoulder_yaw","joint_right_elbow","joint_right_wrist",
        #              "joint_left_hip_roll", "joint_left_hip_yaw","joint_left_hip_pitch","left_knee","joint_left_ankle", 
        #              "joint_right_hip_roll","joint_right_hip_yaw","joint_right_hip_pitch","right_knee","joint_right_ankle"]
        # for i in range(20):
        #     msg.control_modes.append(3)
        #     msg.torques.append(0.0)
        #     msg.speeds.append(0.0)
        #     msg.positions.append(action[3*i])
        #     msg.kps.append(action[3*i + 1])
        #     msg.kds.append(action[3*i + 2])

        # self.motor_pub.publish(msg)
    
    # ========== Status Printing ==========
    # MARK: 状态打印
    
    def _print_status(self):
        """Print status periodically."""
        interval = self.get_status_print_interval()
        if self.loop_cnt % interval == 0:
            elapsed = time.time() - self.loop_start_time
            print(f'\033[94m[Status]\033[0m: {elapsed:.2f}s, loop: {self.loop_cnt}')
    
    def _print_init_info(self):
        """Print initialization information."""
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
        self.get_logger().info(f'\033[96m{self.get_node_name()} Started\033[0m')
        self.get_logger().info(f'\033[93mPolicy:\033[0m {self.policy_name}')
        self.get_logger().info(f'\033[93mModel:\033[0m {self.model_name}')
        if self.controller:
            self.get_logger().info(f'\033[93mNum joints:\033[0m {self.controller.num_joints}')
        else:
            self.get_logger().info(f'\033[93mController:\033[0m Not initialized (set externally)')
        self.get_logger().info(f'\033[93mControl rate:\033[0m {self.control_rate} Hz')
        self.get_logger().info(f'\033[93mJoint state timeout:\033[0m {self._joint_state_timeout}s')
        self.get_logger().info(f'\033[93mTopics:\033[0m')
        self.get_logger().info(f'  Sub: {self.get_topic_joint_states()}, {self.get_topic_imu()}, {self.get_topic_joy()}')
        self.get_logger().info(f'  Pub: {self.get_topic_action()}')
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
    
    # ========== Utility Properties ==========
    
    @property
    def num_joints(self) -> Optional[int]:
        """Get number of joints (None if controller not set)."""
        return self.controller.num_joints if self.controller else None
    
    @property
    def elapsed_time(self) -> float:
        """Get elapsed time since control loop started."""
        return time.time() - self.loop_start_time
    
    def get_current_time(self) -> float:
        """Get current time (for gait phase computation)."""
        return time.time()


def run_controller(controller_class, args=None):
    """
    Convenience function to run a controller.
    
    Args:
        controller_class: A subclass of ControllerBase
        args: Command line arguments
    
    Example:
        if __name__ == '__main__':
            run_controller(PandaController2)
    """
    rclpy.init(args=args)
    
    try:
        node = controller_class()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'\033[91mError: {e}\033[0m')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()
