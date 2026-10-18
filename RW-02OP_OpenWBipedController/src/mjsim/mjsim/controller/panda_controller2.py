"""Panda Robot Controller Node (Using ControllerBase)

A simplified Panda controller that inherits from ControllerBase.
Demonstrates how to create a robot-specific controller with minimal code.

Usage:
  ros2 run mjsim panda_controller2
"""

import numpy as np
import json
from pathlib import Path

from mjsim.controller.controller_base import ControllerBase, ControllerParams, run_controller
from mjsim.policy.policy_controller import PolicyController

# 模型与策略参数
# ==============================================================================
PANDA_DEFAULT_POLICY = 'panda_walk_v1'
PANDA_DEFAULT_MODEL = 'pandav3'
# ==============================================================================

class PandaController2(ControllerBase):
    """
    Panda robot controller using ControllerBase.
    
    Supports two policy modes:
      - WALK: Walking policy (Button A to activate)
      - STAND: Standing policy (Button B to activate)
    
    Only needs to implement:
      - get_node_name(): Return node name
      - get_default_parameters(): Return default params
      - setup_observations(): Set observations for policy
    
    Joystick mapping:
      - Left stick X (axes[0]): left/right velocity (vy)
      - Left stick Y (axes[1]): forward/backward velocity (vx)
      - Right stick X (axes[3]): yaw rotation rate
      - Button A: Switch to WALK mode
      - Button B: Switch to STAND mode
    """
    
    # Velocity command limits
    MAX_VX_positive = 0.8      # m/s forward/backward
    MAX_VX_negative = 0.3
    MAX_VY = 0.3      # m/s left/right
    MAX_YAW = 0.5     # rad/s rotation
    MAX_height = 0.7
    MIN_height = 0.63
    
    # Button indices for mode switching (standard gamepad)
    BUTTON_A = 0  # Switch to walk mode
    BUTTON_B = 1  # Switch to stand mode
    BUTTON_Y = 3  # Record warmup data

    # MARK: 初始化
    def on_init(self):
        """
        Initialize dual controllers and velocity command variables.
        Both controllers are warmed up during initialization.
        """
        # Velocity commands
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_yaw = 0.0
        self.cmd_height = (self.MAX_height + self.MIN_height) / 2
        self.gait_period = 0.7
        self.target_mode = 'STAND'
        
        # Initialize walk controller (warmup happens in PolicyController.__init__)
        self.get_logger().info('\033[96mInitializing WALK controller...\033[0m')
        self.walk_controller = PolicyController(
            policy_name=PANDA_DEFAULT_POLICY,
            model_name=PANDA_DEFAULT_MODEL,
            device=self.device,
            logger=self.get_logger().info
        )
        
        # Initialize stand controller (warmup happens in PolicyController.__init__)
        self.get_logger().info('\033[96mInitializing STAND controller...\033[0m')
        self.stand_controller = PolicyController(
            # policy_name="panda_stand",
            policy_name="panda_stand_LB",
            model_name=PANDA_DEFAULT_MODEL,
            device=self.device,
            logger=self.get_logger().info
        )
        
        # Set default controller to walk
        # self.set_controller(self.walk_controller)
        # self._current_mode = 'WALK'
        self.set_controller(self.stand_controller)
        self._current_mode = 'STAND'
        
        # Track button states for edge detection (only trigger on press, not hold)
        self._last_button_a = 0
        self._last_button_b = 0
        self._last_button_y = 0
        
        # Warmup数据记录
        self._recording_warmup = False
        self._warmup_buffer = []
        self._warmup_target_controller = None  # Which controller to record for

        self.phase_start_time = self.get_current_time()
        
        self.get_logger().info('\033[92mDual controllers initialized. Default mode: WALK\033[0m')
    
    def get_node_name(self) -> str:
        return 'panda_controller2'
    
    # ------------------------------ 默认控制器 ------------------------------
    def get_default_parameters(self) -> ControllerParams:
        return ControllerParams(
            control_rate=50,
            policy_name=PANDA_DEFAULT_POLICY,
            model_name=PANDA_DEFAULT_MODEL,
            device='cuda',
            auto_init_controller=False  # We initialize controllers manually in on_init()
        )
    # MARK: 手柄处理
    def process_joy_data(self):
        """
        Process joystick data for velocity commands and mode switching.
        
        Joystick axes mapping (typical gamepad):
          - axes[0]: Left stick X (left/right) -> vy
          - axes[1]: Left stick Y (up/down) -> vx
          - axes[3]: Right stick X (left/right) -> yaw rate
          - axes[4]: Right stick Y (up/down) -> height
        
        Button mapping:
          - Button A (0): Switch to WALK mode
          - Button B (1): Switch to STAND mode
        """
        if self.joy_data is None:
            return
        
        axes = self.joy_data.axes
        buttons = self.joy_data.buttons
        
        # === 速度与高度控制 ===
        if len(axes) >= 4:
            # Left stick: forward/backward (axes[1]) and left/right (axes[0])
            if axes[1] > 0:
                self.cmd_vx = axes[1] * self.MAX_VX_positive
            else:
                self.cmd_vx = axes[1] * self.MAX_VX_negative
            self.cmd_vy = axes[0] * self.MAX_VY   # Left positive
            
            # Right stick: rotation (axes[3])
            self.cmd_yaw = axes[3] * self.MAX_YAW  # Counter-clockwise positive
            self.cmd_height = axes[4] * (self.MAX_height - self.MIN_height)/2 + (self.MIN_height + self.MAX_height)/2

        
        # === 运动模式切换 ===
        if len(buttons) >= 2:
            # 按键A (0->1 transition): switch to WALK mode
            if buttons[self.BUTTON_A] == 1 and self._last_button_a == 0:
                self.target_mode = 'WALK'
                # if self._current_mode != 'WALK':
                #     self.get_logger().info('\033[93m[Mode Switch] -> WALK\033[0m')
                #     self.set_controller(self.walk_controller)
                #     self.phase_start_time = self.get_current_time()-0.2
                #     self._current_mode = 'WALK'
            
            # 按键B (0->1 transition): switch to STAND mode
            if buttons[self.BUTTON_B] == 1 and self._last_button_b == 0:
                self.target_mode = 'STAND'
                # if self._current_mode != 'STAND':
                #     self.get_logger().info('\033[93m[Mode Switch] -> STAND\033[0m')
                #     self.set_controller(self.stand_controller)
                #     self._current_mode = 'STAND'
            
            self._last_button_a = buttons[self.BUTTON_A]
            self._last_button_b = buttons[self.BUTTON_B]
        
        # Button Y: Start warmup data recording
        if len(buttons) > self.BUTTON_Y:
            if buttons[self.BUTTON_Y] == 1 and self._last_button_y == 0:
                if not self._recording_warmup:
                    self._start_warmup_recording()
            self._last_button_y = buttons[self.BUTTON_Y]
    

    # MARK: 观测更新
    def setup_observations(self) -> bool:
        """
        Set up Panda-specific observations for policy inference.
        """
        # Process joystick input (may switch controller)
        self.process_joy_data()
        
        # Check if controller was switched (q_policy will be None)
        if self.q_policy is None:
            return False  # Skip this cycle, wait for new q_policy
        
        # Get IMU data
        quat = self.imu_data.orientation
        ang_vel = self.imu_data.angular_velocity
        proj_gravity = self.controller._quat_to_proj_gravity(quat)
        
        # Set observations
        self.controller.set_observation("base_ang_vel", 
            np.array([ang_vel.x, ang_vel.y, ang_vel.z]))
        self.controller.set_observation("projected_gravity", proj_gravity)
        self.controller.set_observation("velocity_commands", 
            np.array([self.cmd_vx, self.cmd_vy, self.cmd_yaw]))
        self.controller.set_observation("joint_pos_rel", 
            self.q_policy - self.controller.default_pos_policy)
        self.controller.set_observation("joint_vel_rel", self.dq_policy)
        self.controller.set_observation("last_action", self.controller.last_action)
        
        # Only set gait_phase for WALK mode (stand policy doesn't use it)
        if self._current_mode == 'WALK':
            # pass
            phase = self.get_gait_phase(self.get_current_time() - self.phase_start_time)
            self.controller.set_observation("gait_phase", phase)
        else:
            self.controller.set_observation("height_commands", np.array([self.cmd_height]))
        
        # Record warmup data if in recording mode
        if self._recording_warmup:
            self._record_warmup_frame()
        
        return True
    
    # MARK: Warmup数据记录
    def _start_warmup_recording(self):
        """Start recording warmup data for the current controller."""
        self._recording_warmup = True
        self._warmup_buffer = []
        self._warmup_target_controller = self.controller
        history_length = self.controller.policy_cfg.obs_history_length
        
        self.get_logger().info(
            f'\033[93m[Warmup Recording] Started for {self.controller.policy_name}, '
            f'recording {history_length} frames...\033[0m'
        )
    
    def _record_warmup_frame(self):
        """Record one frame of observation data."""
        if self._warmup_target_controller != self.controller:
            # Controller changed during recording, abort
            self.get_logger().warn('\033[91m[Warmup Recording] Aborted: controller changed\033[0m')
            self._recording_warmup = False
            self._warmup_buffer = []
            return
        
        # Get observation keys from policy
        obs_keys = list(self.controller.policy.observations.keys())
        
        # Record current observation values
        frame_data = {}
        for key in obs_keys:
            value = self.controller.policy.observations.get(key)
            if value is not None:
                frame_data[key] = value.tolist()
        
        self._warmup_buffer.append(frame_data)
        
        # Check if recording is complete
        history_length = self.controller.policy_cfg.obs_history_length
        if len(self._warmup_buffer) >= history_length:
            self._save_warmup_data()
            self._recording_warmup = False
            self._warmup_buffer = []
    
    def _save_warmup_data(self):
        """Save recorded warmup data to model_info.json."""
        policy_path = self._warmup_target_controller.policy_cfg.policy_path
        config_file = policy_path / 'model_info.json'
        
        if not config_file.exists():
            self.get_logger().error(f'\033[91m[Warmup Recording] Config file not found: {config_file}\033[0m')
            return
        
        try:
            # Read existing config
            with open(config_file, 'r') as f:
                config_data = json.load(f)
            
            # Update or add warmup_data field
            config_data['warmup_data'] = self._warmup_buffer
            
            # Write back
            with open(config_file, 'w') as f:
                json.dump(config_data, f, indent=4)
            
            self.get_logger().info(
                f'\033[92m[Warmup Recording] Saved {len(self._warmup_buffer)} frames to '
                f'{config_file}\033[0m'
            )
        except Exception as e:
            self.get_logger().error(f'\033[91m[Warmup Recording] Failed to save: {e}\033[0m')
    
    # MARK: Gait
    def get_gait_phase(self, gait_time: float) -> np.ndarray:
        """
        Returns:
            [sin(phi), cos(phi)]
        """
        phi = (gait_time % self.gait_period) / self.gait_period  # [0, 1]
        angle = phi * 2 * np.pi                                  # [0, 2*pi]
        return np.array([np.sin(angle), np.cos(angle)], dtype=np.float32)
    
    def policy_switch_manager(self):
        """策略切换管理器"""
        if self._current_mode != self.target_mode:
            if self.target_mode == 'WALK':
                self.set_controller(self.walk_controller)
                self.phase_start_time = self.get_current_time()-0.45
                self._current_mode = 'WALK'
            elif self.target_mode == 'STAND':
                gait_time = self.get_current_time() - self.phase_start_time
                phi = (gait_time % self.gait_period) / self.gait_period  # [0, 1]
                if phi>0.2 and phi<0.4:
                    self.set_controller(self.stand_controller)
                    self._current_mode = 'STAND'
                    print("切换phi = ", phi)
    
    def on_action_published(self):
        """在action发送之后执行策略切换检查"""
        self.policy_switch_manager()




def main(args=None):
    run_controller(PandaController2, args)


if __name__ == '__main__':
    main()
