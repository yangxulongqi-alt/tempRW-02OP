"""
Policy Controller

A high-level controller that encapsulates:
  - Policy neural network
  - Policy configuration (from model_info.json)
  - MJCF model configuration (from ModelConfig)
  - Joint order mapping between policy and XML

External interface is simple:
  1. Initialize: controller = PolicyController(policy_name, model_name)
  2. Set observations: controller.set_observation(name, value)
  3. Compute action: action = controller.compute_action()

Usage:
    # Initialize
    controller = PolicyController(
        policy_name='panda_walk',
        model_name='panda',
        device='cuda'
    )
    
    # In control loop
    controller.set_observation("base_ang_vel", ang_vel)
    controller.set_observation("projected_gravity", proj_gravity)
    controller.set_observation("velocity_commands", vel_cmd)
    controller.set_observation("joint_pos_rel", q - default_pos)
    controller.set_observation("joint_vel_rel", dq)
    controller.set_observation("last_action", last_action)
    controller.set_observation("gait_phase", phase)
    
    # Get MIT format action: [pos0, kp0, kd0, pos1, kp1, kd1, ...]
    action = controller.compute_action()
"""

import numpy as np
import json
from typing import Optional, Callable, List
from dataclasses import dataclass

from mjsim.policy.policy import Policy
from mjsim.policy.policyCfg import load_policy_config, PolicyConfig
from mjsim.cfgs import MJCF_MODEL_REGISTRY, POLICY_REGISTRY, ModelConfig


@dataclass 
class ControlOutput:
    """Control output structure"""
    mit_action: np.ndarray       # [pos0, kp0, kd0, pos1, kp1, kd1, ...] in XML order
    target_pos_xml: np.ndarray   # Target positions in XML order
    raw_action: np.ndarray       # Raw policy output (policy order)
    

class PolicyController:
    """
    自动处理策略与控制器的匹配，包括关节顺序、电机方向、observation_scale、default_position
    、action scale等处理
    
    简洁的接口:
      1. 通过注册的`策略名`和`mjcf模型名`初始化
      2. 设置观测值
      3. 计算控制动作
    """
    
    def __init__(
        self,
        policy_name: str,
        model_name: str,
        device: str = 'cuda',
        logger: Optional[Callable] = None
    ):
        """
        Initialize PolicyController.
        
        Args:
            policy_name: POLICY_REGISTR中注册的策略名 (e.g., 'panda_walk')
            model_name: MJCF_MODEL_REGISTRY中注册的模型名 (e.g., 'panda')
            device: 运行设备名'cuda' or 'cpu'
            logger: Optional logging function (e.g., node.get_logger().info)
        """
        self.policy_name = policy_name
        self.model_name = model_name
        self.device = device
        self._logger = logger
        
        # Load configurations
        self._log(f'\033[96m{"="*60}\033[0m')
        self._log(f'\033[96mPolicyController Initializing\033[0m')
        self._log(f'\033[93mPolicy:\033[0m {policy_name}')
        self._log(f'\033[93mModel:\033[0m {model_name}')
        
        # Load policy config (from model_info.json)
        self.policy_cfg: PolicyConfig = load_policy_config(policy_name)
        self.num_joints = self.policy_cfg.num_joints
        
        # Load model config (from cfgs.py)
        self.model_cfg: ModelConfig = MJCF_MODEL_REGISTRY.get(model_name)
        if self.model_cfg is None:
            raise ValueError(f"实体模型 '{model_name}' 未注册！")
        
        # Initialize policy network
        policy_path = POLICY_REGISTRY[policy_name] / 'policy.pt'
        obs_specs = self.policy_cfg.get_obs_specs()
        
        self.policy = Policy(
            model_path=str(policy_path),
            device=device,
            obs_specs=obs_specs,
            history_length=self.policy_cfg.obs_history_length
        )
        self.warm_up()
        
        # Build joint order mapping
        self._init_joint_mapping()
        
        # Pre-compute action joint mask for efficiency (MUST be before _init_control_params)
        self.action_joint_mask = np.array([joint_info.is_action_joint for joint_info in self.policy_cfg.joint_info])
        
        # Convert control parameters to XML order
        self._init_control_params()
        
        # State
        num_action_joints = sum(1 for joint_info in self.policy_cfg.joint_info if joint_info.is_action_joint)
        self.last_action = np.zeros(num_action_joints)
        self._xml_to_policy_idx: Optional[np.ndarray] = None
        
        self._print_control_params()
        self._log(f'\033[92mPolicyController Ready!\033[0m')
        self._log(f'\033[96m{"="*60}\033[0m')
    
    # MARK: 初始化
    # ========================================
    
    def _init_joint_mapping(self):
        """Initialize joint order mapping between policy and XML."""
        self.policy_joint_names = self.policy_cfg.joint_names
        self.xml_motor_names = self.model_cfg.motor_names if self.model_cfg else []
        self.motor_dir = np.array(self.model_cfg.motor_dir) if self.model_cfg.motor_dir else np.ones(self.num_joints)
        
        # Build policy->xml mapping
        n = len(self.policy_joint_names)
        self._policy_to_xml_idx = np.zeros(n, dtype=int)
        
        self._log(f'\033[92mBuilding policy->xml mapping:\033[0m')
        for policy_i, name in enumerate(self.policy_joint_names):
            if name in self.xml_motor_names:
                xml_i = self.xml_motor_names.index(name)
                self._policy_to_xml_idx[policy_i] = xml_i
                self._log(f'  policy[{policy_i:2d}] {name:30s} -> xml[{xml_i:2d}]')
            else:
                self._log(f'  policy[{policy_i:2d}] {name:30s} NOT FOUND in xml!', 'warn')
    
    def _init_control_params(self):
        """Convert control parameters from policy order to XML order."""
        self.kp_xml = self._reorder_policy_to_xml(self.policy_cfg.stiffness)
        self.kd_xml = self._reorder_policy_to_xml(self.policy_cfg.damping)
        self.default_pos_xml = self._reorder_policy_to_xml(self.policy_cfg.default_pos)
        self.action_scale_xml = self._reorder_policy_to_xml(self.policy_cfg.action_scale)
        self.action_offset_xml = self._reorder_policy_to_xml(self.policy_cfg.action_offset)
    
    def _print_control_params(self):
        """Print control parameters."""
        np.set_printoptions(precision=3, suppress=True)
        self._log(f'\033[93mKp (xml order):\033[0m {self.kp_xml}')
        self._log(f'\033[93mKd (xml order):\033[0m {self.kd_xml}')
        self._log(f'\033[93mDefault Pos (xml order):\033[0m {self.default_pos_xml}')
    
    # ========== Logging ==========
    
    def _log(self, msg: str, level: str = 'info'):
        """Log message using provided logger or print."""
        if self._logger:
            if level == 'warn':
                self._logger(f'\033[93m[WARN]\033[0m {msg}')
            else:
                self._logger(msg)
        else:
            if level == 'warn':
                print(f'\033[93m[WARN]\033[0m {msg}')
            else:
                print(msg)
    
    # MARK:关节映射
    # ========================================
    
    def _reorder_policy_to_xml(self, policy_arr: np.ndarray, action_joints_only: bool = False) -> np.ndarray:
        """Reorder array from policy order to XML order.
        
        Args:
            policy_arr: Input array in policy order
            action_joints_only: If True, input contains only action joints.
                               If False, input contains all joints.
        """
        # Create result array with size equal to number of joints in XML
        result = np.zeros(len(self._policy_to_xml_idx))
        
        if action_joints_only:
            # Input array only contains action joints
            action_joint_indices = np.where(self.action_joint_mask)[0]
            for idx, policy_i in enumerate(action_joint_indices):
                if idx < len(policy_arr):  # Safety check
                    xml_i = self._policy_to_xml_idx[policy_i]
                    result[xml_i] = policy_arr[idx]
        else:
            # Input array contains all joints
            for policy_i in range(len(self._policy_to_xml_idx)):
                if policy_i < len(policy_arr):
                    xml_i = self._policy_to_xml_idx[policy_i]
                    result[xml_i] = policy_arr[policy_i]
        
        return result
    
    def _reorder_xml_to_policy(self, xml_arr: np.ndarray) -> np.ndarray:
        """Reorder array from XML order to policy order (all joints)."""
        # Create result array with size equal to number of all joints
        result = np.zeros(len(self._policy_to_xml_idx))
        
        # Use the original policy->xml mapping to reverse the process
        for policy_i in range(len(self._policy_to_xml_idx)):
            xml_i = self._policy_to_xml_idx[policy_i]
            if xml_i < len(xml_arr):  # Safety check
                result[policy_i] = xml_arr[xml_i]
        return result
    
    def build_xml_to_policy_mapping(self, msg_names: List[str]):
        """
        Build mapping from JointState message order to policy order.
        Call this when receiving the first JointState message.
        
        Args:
            msg_names: Joint names from JointState message
        """
        # Initialize mapping array for all policy joints
        n = len(self.policy_joint_names)
        self._xml_to_policy_idx = np.zeros(n, dtype=int)
        
        self._log(f'\033[92mBuilding xml->policy mapping (for q/dq):\033[0m')
        for policy_i, name in enumerate(self.policy_joint_names):
            # Only map action joints
            if self.action_joint_mask[policy_i] and name in msg_names:
                xml_i = msg_names.index(name)
                self._xml_to_policy_idx[policy_i] = xml_i
                self._log(f'  xml[{xml_i:2d}] {name:30s} -> policy[{policy_i:2d}]')
            elif not self.action_joint_mask[policy_i]:
                # Non-action joints are not mapped but we set to -1 to indicate they're skipped
                self._xml_to_policy_idx[policy_i] = -1
            else:
                self._xml_to_policy_idx[policy_i] = -1  # Indicate joint not found
                self._log(f'  {name} NOT FOUND in msg!', 'warn')
    
    # MARK: 外部接口
    # ========================================
    
    def set_observation(self, key: str, value: np.ndarray):
        """
        Set observation value.
        
        Args:
            key: Observation name (e.g., 'base_ang_vel', 'projected_gravity')
            value: Observation value as numpy array
        """
        self.policy.set_observation(key, value)
    
    def set_observations_from_state(
        self,
        q_xml: np.ndarray,
        dq_xml: np.ndarray,
        quat: tuple,
        ang_vel: tuple,
        vel_cmd: np.ndarray = None,
        gait_phase: np.ndarray = None
    ):
        """
        Convenience method to set all observations from robot state.
        
        Args:
            q_xml: Joint positions in XML order
            dq_xml: Joint velocities in XML order
            quat: IMU quaternion (x, y, z, w)
            ang_vel: IMU angular velocity (x, y, z)
            vel_cmd: Velocity command [vx, vy, wz] (optional, default zeros)
            gait_phase: Gait phase [sin, cos] (optional, computed from time if not provided)
        """
        # Convert q/dq from XML to policy order with motor direction
        q_with_dir = np.array(q_xml) * self.motor_dir
        dq_with_dir = np.array(dq_xml) * self.motor_dir
        q_policy = self._reorder_xml_to_policy(q_with_dir)
        dq_policy = self._reorder_xml_to_policy(dq_with_dir)
        
        # Compute projected gravity from quaternion
        proj_gravity = self._quat_to_proj_gravity(quat)
        
        # Set observations
        self.policy.set_observation("base_ang_vel", np.array([ang_vel[0], ang_vel[1], ang_vel[2]]))
        self.policy.set_observation("projected_gravity", proj_gravity)
        self.policy.set_observation("velocity_commands", vel_cmd if vel_cmd is not None else np.zeros(3))
        # Use default positions for all joints (observations need all joints)
        self.policy.set_observation("joint_pos_rel", q_policy - self.policy_cfg.default_pos)
        self.policy.set_observation("joint_vel_rel", dq_policy)
        self.policy.set_observation("last_action", self.last_action)
        
        if gait_phase is not None:
            self.policy.set_observation("gait_phase", gait_phase)
    
    # MARK: Action
    def compute_action(self) -> np.ndarray:
        """
        Compute control action from current observations.
        
        Returns:
            MIT format action array: [pos0, kp0, kd0, pos1, kp1, kd1, ...]
            in XML motor order, ready to send to robot/simulator.
            Non-action joints will have -1 as their target position.
        """
        # Run policy inference
        raw_action = self.policy.step()
        self.last_action = raw_action.copy()
        
        # Get action joints only for scaling
        action_scale_filtered = self.policy_cfg.action_scale[self.action_joint_mask]
        action_offset_filtered = self.policy_cfg.action_offset[self.action_joint_mask]
        
        # Scale action to target position (policy order) - only for action joints
        target_pos_policy = raw_action * action_scale_filtered + action_offset_filtered
        
        # Convert to XML order and apply motor direction
        target_pos_xml = self._reorder_policy_to_xml(target_pos_policy, action_joints_only=True) * self.motor_dir
        
        # Filter joints based on is_action_joint and set non-action joints to -1
        filtered_target_pos_xml = self._filter_action_joints(target_pos_xml)
        
        # Build MIT control command
        action = self._build_mit_action(filtered_target_pos_xml)
        
        return action
    
    def compute_action_detailed(self) -> ControlOutput:
        """
        Compute control action with detailed output.
        
        Returns:
            ControlOutput with mit_action, target_pos_xml, and raw_action.
            Non-action joints will have -1 as their target position.
        """
        raw_action = self.policy.step()
        self.last_action = raw_action.copy()
        
        # Get action joints only for scaling
        action_scale_filtered = self.policy_cfg.action_scale[self.action_joint_mask]
        action_offset_filtered = self.policy_cfg.action_offset[self.action_joint_mask]
        
        target_pos_policy = raw_action * action_scale_filtered + action_offset_filtered
        target_pos_xml = self._reorder_policy_to_xml(target_pos_policy, action_joints_only=True) * self.motor_dir
        
        # Filter joints based on is_action_joint and set non-action joints to -1
        filtered_target_pos_xml = self._filter_action_joints(target_pos_xml)
        mit_action = self._build_mit_action(filtered_target_pos_xml)
        
        return ControlOutput(
            mit_action=mit_action,
            target_pos_xml=filtered_target_pos_xml,
            raw_action=raw_action
        )
    
    # MARK: 辅助函数
    # ========== Helper Methods ==========
    
    def _filter_action_joints(self, target_pos: np.ndarray) -> np.ndarray:
        """
        Filter joints based on is_action_joint flag.
        Set non-action joints to 0.0.
        
        Args:
            target_pos: Target joint positions in XML order
            
        Returns:
            Filtered target positions with 0.0 for non-action joints
        """
        filtered_pos = target_pos.copy()
        
        # Iterate through joint info and set non-action joints to -1
        for i, joint_info in enumerate(self.policy_cfg.joint_info):
            # Map policy order index to XML order index
            xml_idx = self._policy_to_xml_idx[i]
            if not joint_info.is_action_joint:
                # filtered_pos[xml_idx] = -1.0
                filtered_pos[xml_idx] = 0.0
                
        return filtered_pos
    
    def _build_mit_action(self, target_pos: np.ndarray) -> np.ndarray:
        """
        Build MIT control action from target positions.
        
        Args:
            target_pos: Target joint positions in XML order
        
        Returns:
            MIT action: [pos0, kp0, kd0, pos1, kp1, kd1, ...]
        """
        n = min(len(target_pos), self.num_joints)
        action = np.zeros(3 * self.num_joints)
        
        for i in range(n):
            action[i * 3] = target_pos[i]       # pos_des
            action[i * 3 + 1] = self.kp_xml[i]  # kp
            action[i * 3 + 2] = self.kd_xml[i]  # kd
        
        return action
    
    def _quat_to_proj_gravity(self, quat) -> np.ndarray:
        """
        Convert quaternion to projected gravity vector.
        
        Args:
            quat: Quaternion as (x, y, z, w) tuple or object with .x, .y, .z, .w
        """
        if hasattr(quat, 'x'):
            x, y, z, w = quat.x, quat.y, quat.z, quat.w
        else:
            x, y, z, w = quat[0], quat[1], quat[2], quat[3]
        
        # Rotate [0, 0, -1] by inverse quaternion
        gx = 2.0 * (x * z - w * y)
        gy = 2.0 * (y * z + w * x)
        gz = w * w - x * x - y * y + z * z
        return np.array([-gx, -gy, -gz])
    
    # ========== Properties ==========
    
    @property
    def default_pos_policy(self) -> np.ndarray:
        """Get default positions in policy order (all joints, for observations)."""
        return self.policy_cfg.default_pos
    
    @property
    def default_pos_action_only(self) -> np.ndarray:
        """Get default positions for action joints only."""
        return self.policy_cfg.default_pos[self.action_joint_mask]
    
    @property
    def xml_path(self) -> str:
        """Get XML model path."""
        return str(self.model_cfg.xml_path)
    
    def reset(self):
        """Reset controller state."""
        self.policy.reset()
        num_action_joints = sum(1 for joint_info in self.policy_cfg.joint_info if joint_info.is_action_joint)
        self.last_action = np.zeros(num_action_joints)
        self._xml_to_policy_idx = None
    
    def warm_up(self, iterations: int = 10):
        """
        Warm up the policy with historical data from model_info.json if available,
        otherwise use default zero observations.
        
        Args:
            iterations: Number of warmup iterations (used only for default warmup)
        """
        # Try to load warmup_data from model_info.json
        config_file = self.policy_cfg.policy_path / 'model_info.json'
        warmup_data = None
        
        if config_file.exists():
            try:
                with open(config_file, 'r') as f:
                    config_data = json.load(f)
                warmup_data = config_data.get('warmup_data')
            except Exception as e:
                self._log(f'\033[93m[Warmup] Failed to read config: {e}\033[0m', 'warn')
        
        obs_keys = list(self.policy.observations.keys())
        
        # Print warmup configuration
        self._log(f'\033[96m{"="*50}\033[0m')
        self._log(f'\033[96mPolicy Warmup Configuration\033[0m')
        self._log(f'\033[93mObs keys:\033[0m {obs_keys}')
        
        if warmup_data and len(warmup_data) > 0:
            # Use historical warmup data
            self._log(f'\033[93mMode:\033[0m Using recorded warmup_data ({len(warmup_data)} frames)')
            self._warmup_with_recorded_data(warmup_data, obs_keys)
        else:
            # Fall back to default policy warmup
            self._log(f'\033[93mMode:\033[0m Default warmup ({iterations} iterations)')
            self.policy.warm_up(iterations)
            return  # policy.warm_up already prints "done" message
        
        self._log(f'\033[92mPolicy warmup done!\033[0m')
        self._log(f'\033[96m{"="*50}\033[0m')
    
    def _warmup_with_recorded_data(self, warmup_data: list, obs_keys: list):
        """
        Warm up using recorded observation data.
        
        Args:
            warmup_data: List of observation dictionaries from model_info.json
            obs_keys: List of observation keys in the policy
        """
        num_action_joints = sum(1 for j in self.policy_cfg.joint_info if j.is_action_joint)
        action = np.zeros(num_action_joints)
        
        for frame_data in warmup_data:
            # Set observations from recorded data
            for key in obs_keys:
                if key in frame_data:
                    value = np.array(frame_data[key])
                    self.policy.set_observation(key, value)
                elif key == "last_action":
                    # Use current action for last_action if not in recorded data
                    self.policy.set_observation(key, action)
            
            self.policy.obs2buffer()
            self.policy.infer()
            action = self.policy.get_action()


# ========== Convenience Functions ==========

def create_policy_controller(
    policy_name: str,
    model_name: str,
    device: str = 'cuda',
    logger: Optional[Callable] = None
) -> PolicyController:
    """
    Factory function to create a PolicyController.
    
    Args:
        policy_name: Policy name in POLICY_REGISTRY
        model_name: Model name in MODEL_REGISTRY
        device: 'cuda' or 'cpu'
        logger: Optional logging function
    
    Returns:
        Configured PolicyController instance
    """
    return PolicyController(
        policy_name=policy_name,
        model_name=model_name,
        device=device,
        logger=logger
    )
