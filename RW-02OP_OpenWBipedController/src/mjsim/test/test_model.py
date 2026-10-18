"""
Test script to read model information from XML

Usage:
    python3 test_model.py [model_name]
    
Example:
    python3 test_model.py honghu
    python3 test_model.py panda
"""

import sys
import os
import numpy as np

# Add package path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import mujoco


def get_model_info(model_name: str):
    """
    Read model info from XML file by model name.
    
    Args:
        model_name: Model name registered in MJCF_MODEL_REGISTRY (e.g., 'honghu', 'panda')
    """
    # Import here to handle ROS package path
    try:
        from mjsim.cfgs import MJCF_MODEL_REGISTRY, get_model_path
    except ImportError:
        # Fallback for direct run without ROS
        print("\033[91mError: Cannot import mjsim.cfgs. Run with ROS environment.\033[0m")
        return
    
    # Get model config
    if model_name not in MJCF_MODEL_REGISTRY:
        print(f"\033[91mError: Model '{model_name}' not found.\033[0m")
        print(f"Available models: {list(MJCF_MODEL_REGISTRY.keys())}")
        return
    
    model_cfg = MJCF_MODEL_REGISTRY[model_name]
    xml_path = str(model_cfg.xml_path)
    
    print(f"\033[96m{'='*60}\033[0m")
    print(f"\033[96mModel Info: {model_name}\033[0m")
    print(f"\033[93mXML Path:\033[0m {xml_path}")
    
    # Check if file exists
    if not os.path.exists(xml_path):
        print(f"\033[91mError: XML file not found: {xml_path}\033[0m")
        return
    
    # Load MuJoCo model
    model = mujoco.MjModel.from_xml_path(xml_path)
    
    # Get num_joints (number of actuators)
    num_joints = model.nu
    print(f"\033[93mnum_joints:\033[0m {num_joints}")
    
    # Get motor_names and motor_dir
    motor_names = []
    motor_dir = []
    
    print(f"\033[93mmotor_names & motor_dir:\033[0m")
    for i in range(num_joints):
        # Get actuator name
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator_{i}"
        motor_names.append(name)
        
        # Get joint axis direction
        jnt_id = model.actuator_trnid[i, 0]
        axis = model.jnt_axis[jnt_id]
        axis_sign = int(np.sign(axis[np.argmax(np.abs(axis))]))
        motor_dir.append(axis_sign)
        
        print(f"  [{i:2d}] {name:40s} dir: {axis_sign:+d}")
    
    print(f"\033[96m{'='*60}\033[0m")
    
    # Output in Python format for easy copy-paste
    print(f"\n\033[92m# 直接粘贴的形式:\033[0m")
    print(f"\033[92mnum_joints\033[0m={num_joints},")
    print(f"\033[92mmotor_names\033[0m={motor_names},")
    print(f"\033[92mmotor_dir\033[0m={motor_dir},")


def main():
    if len(sys.argv) < 2:
        model_name = 'honghu'  # Default model
        print(f"No model specified, using default: '{model_name}'")
    else:
        model_name = sys.argv[1]
    
    get_model_info(model_name)


if __name__ == '__main__':
    main()
