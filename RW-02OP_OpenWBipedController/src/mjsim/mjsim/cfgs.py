import os
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import mujoco

# ROS工作空间目录
__WS_DIR = Path(get_package_share_directory('mjsim')).parent.parent.parent.parent
__ASSET_DIR = __WS_DIR / 'assets'
POLICY_DIR = Path(get_package_share_directory('mjsim')) /'models' / 'policies'
LOG_DIR = __WS_DIR / 'logs'


@dataclass
class ModelConfig:
    """Model configuration with default control parameters"""
    xml_path: Path
    num_joints: int = 0
    default_kp: np.ndarray = field(default_factory=list)
    default_kd: np.ndarray = field(default_factory=list)
    pd_stand_pos: np.ndarray = field(default_factory=list)
    motor_names: list = field(default_factory=list)  # Policy order motor names
    motor_dir: list = field(default_factory=list)    # Joint direction (+1 or -1)


# MARK: 注册
# Model registry: name -> ModelConfig
MJCF_MODEL_REGISTRY = {
    'nezhav1': ModelConfig(
        xml_path=__ASSET_DIR / 'nezha' / 'models' / 'nezhav1' / 'mjcf' / 'mjmodel.xml',
        num_joints=8,
        default_kp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        default_kd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        pd_stand_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        motor_names=['abad_L_Joint', 'hip_L_Joint', 'knee_L_Joint', 'wheel_L_Joint', 'abad_R_Joint', 'hip_R_Joint', 'knee_R_Joint', 'wheel_R_Joint'],
        motor_dir=[1, -1, -1, 1, -1, -1, -1, 1],
    ),
}

POLICY_REGISTRY = {
    
}











# MARK: utils
# 名称-->xml配置
def get_model_config(name: str) -> Optional[ModelConfig]:
    """Get model config, auto-fill motor_names and motor_dir if empty."""
    config = MJCF_MODEL_REGISTRY.get(name)
    if config is None:
        return None
    
    # Auto-fill motor_names if empty
    if not config.motor_names:
        motor_names = get_model_motor_names(name)
        if motor_names:
            config.motor_names = motor_names
    
    # Auto-fill motor_dir if empty
    if not config.motor_dir:
        motor_dir = get_model_motor_directions(name)
        if motor_dir:
            config.motor_dir = motor_dir
    print("=="*20)
    print(config)
    return config


def get_model_path(name: str) -> str:
    """ 通过名称获取xml_path """
    # Check if it's a registered model name
    if name in MJCF_MODEL_REGISTRY:
        return str(MJCF_MODEL_REGISTRY[name].xml_path)
    
    # Check if it's a direct path
    if os.path.exists(name):
        return name
    
    # List available models
    available = ', '.join(MJCF_MODEL_REGISTRY.keys())
    raise ValueError(f"Model '{name}' not found. Available: {available}")


def list_models() -> list:
    """Return list of available model names"""
    return list(MJCF_MODEL_REGISTRY.keys())



# motor顺序
def get_model_motor_names(name: str) -> Optional[list]:
    """Get actuator/motor names from a MuJoCo model.
    
    Args:
        name: Model name registered in MJCF_MODEL_REGISTRY or direct path to xml file.
        
    Returns:
        List of actuator names in the model, or None if model not found.
    """
    try:
        model_path = get_model_path(name)
    except ValueError:
        return None
    model = mujoco.MjModel.from_xml_path(model_path)
    names = []
    for i in range(model.nu):  # nu is the number of actuators
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        names.append(actuator_name or f"actuator_{i}")
    return names


def get_model_motor_directions(name: str) -> Optional[list]:
    """Get motor/actuator directions from a MuJoCo model based on joint axis.
    
    The direction is determined by the sign of the primary axis component.
    Positive axis -> +1, Negative axis -> -1.
    
    Args:
        name: Model name registered in MJCF_MODEL_REGISTRY or direct path to xml file.
        
    Returns:
        List of directions (+1 or -1) for each actuator, or None if model not found.
    """
    try:
        model_path = get_model_path(name)
    except ValueError:
        return None
    
    model = mujoco.MjModel.from_xml_path(model_path)
    directions = []
    
    for i in range(model.nu):
        # Get the transmission target (joint id) for this actuator
        # actuator_trnid[i, 0] is the joint id for position/velocity actuators
        jnt_id = model.actuator_trnid[i, 0]
        
        # Get joint axis (shape: nj x 3)
        axis = model.jnt_axis[jnt_id]  # [x, y, z]
        
        # Find the primary axis (the component with largest absolute value)
        primary_idx = np.argmax(np.abs(axis))
        primary_value = axis[primary_idx]
        
        # Direction based on sign of primary axis
        directions.append(1 if primary_value >= 0 else -1)
    
    return directions

