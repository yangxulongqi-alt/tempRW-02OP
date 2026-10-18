
import numpy as np

from mjsim.controller.controller_base import ControllerBase, ControllerParams, run_controller


class HonghuController(ControllerBase):
    """
    Honghu控制器
    
    Only needs to implement:
      - get_node_name(): ROS节点名
      - get_default_parameters(): 提供默认控制器参数
      - setup_observations(): 设置观测值（不同控制器要求不同）
    """
    
    def get_node_name(self) -> str:
        return 'honghu_controller'
    
    def get_default_parameters(self) -> ControllerParams:
        return ControllerParams(
            control_rate=50,
            policy_name='panda_walk',
            model_name='panda',
            device='cuda'
        )
    
    def setup_observations(self) -> bool:
        """
        Set up Panda-specific observations for policy inference.
        """
        # Check if q_policy is valid
        if self.q_policy is None:
            return False  # Skip this cycle
        
        # Get IMU data
        quat = self.imu_data.orientation
        ang_vel = self.imu_data.angular_velocity
        proj_gravity = self.controller._quat_to_proj_gravity(quat)
        
        # Set observations
        self.controller.set_observation("base_ang_vel", 
            np.array([ang_vel.x, ang_vel.y, ang_vel.z]))
        self.controller.set_observation("projected_gravity", proj_gravity)
        self.controller.set_observation("velocity_commands", 
            np.array([1.0, 0.0, 0.0]))
        self.controller.set_observation("joint_pos_rel", 
            self.q_policy - self.controller.default_pos_policy)
        self.controller.set_observation("joint_vel_rel", self.dq_policy)
        self.controller.set_observation("last_action", self.controller.last_action)
        self.controller.set_observation("gait_phase", 
            self.controller.get_realtime_phase(self.get_current_time(), 0.6))
        
        return True




def main(args=None):
    run_controller(PandaController2, args)


if __name__ == '__main__':
    main()
