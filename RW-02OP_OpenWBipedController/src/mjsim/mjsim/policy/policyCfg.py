"""
策略配置文件读取

读取策略目录下的 model_info.json 并返回结构化配置。
"""

import json
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

from mjsim.cfgs import POLICY_REGISTRY


@dataclass
class JointInfo:
    """Single joint configuration"""
    index: int
    name: str
    is_action_joint: bool
    direction: int
    default_pos: float
    stiffness: float
    damping: float
    action_scale: Optional[float]
    action_offset: Optional[float]
    action_clip: Optional[float] = None


@dataclass
class ObservationInfo:
    """Single observation configuration"""
    name: str
    dim_total: int
    dim_single: int
    scale: float
    clip: Optional[float] = None


@dataclass
class ActionTerms:
    """Action terms configuration"""
    joint_names: List[str]
    joint_ids: List[int]


# MARK: 结构化配置文件
@dataclass
class PolicyConfig:
    """Complete policy configuration"""
    num_joints: int
    joint_info: List[JointInfo]
    action_terms: ActionTerms
    observations: List[ObservationInfo]
    obs_history_length: int
    policy_path: Path
    
    # Derived arrays for easy access
    default_pos: np.ndarray = field(default_factory=lambda: np.array([]))
    stiffness: np.ndarray = field(default_factory=lambda: np.array([]))
    damping: np.ndarray = field(default_factory=lambda: np.array([]))
    action_scale: np.ndarray = field(default_factory=lambda: np.array([]))
    action_offset: np.ndarray = field(default_factory=lambda: np.array([]))
    joint_names: List[str] = field(default_factory=list)
    
    def __post_init__(self):
        """Build derived arrays from joint_info"""
        n = len(self.joint_info)
        self.default_pos = np.array([j.default_pos for j in self.joint_info])
        self.stiffness = np.array([j.stiffness for j in self.joint_info])
        self.damping = np.array([j.damping for j in self.joint_info])
        self.action_scale = np.array([j.action_scale for j in self.joint_info])
        self.action_offset = np.array([j.action_offset for j in self.joint_info])
        self.joint_names = [j.name for j in self.joint_info]
    
    def get_obs_specs(self) -> List[tuple]:
        """Get observation specs as list of (name, dim, scale) tuples"""
        return [(obs.name, obs.dim_single, obs.scale) for obs in self.observations]


def load_policy_config(policy_name: str) -> PolicyConfig:
    """
    Load policy configuration from model_info.json
    
    Args:
        policy_name: Policy name registered in POLICY_REGISTRY (e.g., 'panda_walk')
    
    Returns:
        PolicyConfig: Structured policy configuration
    
    Raises:
        ValueError: If policy not found
        FileNotFoundError: If model_info.json not found
    """
    if policy_name not in POLICY_REGISTRY:
        available = ', '.join(POLICY_REGISTRY.keys())
        raise ValueError(f"Policy '{policy_name}' not found. Available: {available}")
    
    policy_path = POLICY_REGISTRY[policy_name]
    config_file = policy_path / 'model_info.json'
    
    if not config_file.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    with open(config_file, 'r') as f:
        data = json.load(f)
    
    # Parse joint_info
    joint_info = []
    for j in data['joint_info']:
        joint_info.append(JointInfo(
            index=j['index'],
            name=j['name'],
            is_action_joint=j.get('is_action_joint', True),  # 默认为 True 如果字段不存在
            direction=j['direction'],
            default_pos=j['default_pos'],
            stiffness=j['stiffness'],
            damping=j['damping'],
            action_scale=j['action_scale'],
            action_offset=j['action_offset'],
            action_clip=j.get('action_clip')
        ))
    
    # Parse action_terms
    action_data = data['action_terms']['JointPositionAction']
    action_terms = ActionTerms(
        joint_names=action_data['joint_names'],
        joint_ids=action_data['joint_ids']
    )
    
    # Parse observations
    observations = []
    for obs in data.get('observations', []):
        observations.append(ObservationInfo(
            name=obs['name'],
            dim_total=obs['dim_total'],
            dim_single=obs['dim_single'],
            scale=obs['scale'],
            clip=obs.get('clip')
        ))
    
    obs_history_length = data.get('obs_history_length', 10)
    
    config = PolicyConfig(
        num_joints=data['num_joints'],
        joint_info=joint_info,
        action_terms=action_terms,
        observations=observations,
        obs_history_length=obs_history_length,
        policy_path=policy_path
    )
    
    _print_config(policy_name, config)
    return config


def _print_config(name: str, cfg: PolicyConfig):
    """Print colored config summary"""
    print(f"\033[96m{'='*50}\033[0m")
    print(f"\033[91mPolicyConfig Loaded: {name}\033[0m")
    print(f"\033[93mPath:\033[0m {cfg.policy_path}")
    print(f"\033[93mNum joints:\033[0m {cfg.num_joints}")
    print(f"\033[93mJoints:\033[0m")
    for j in cfg.joint_info[:5]:
        print(f"  [{j.index:2d}] {j.name}: kp={j.stiffness:.2f}, kd={j.damping:.2f}, default={j.default_pos:.2f}")
    if cfg.num_joints > 5:
        print(f"  ... ({cfg.num_joints - 5} more)")
    print(f"\033[96m{'='*50}\033[0m")


def get_policy_path(policy_name: str) -> Path:
    """Get policy directory path"""
    if policy_name not in POLICY_REGISTRY:
        available = ', '.join(POLICY_REGISTRY.keys())
        raise ValueError(f"Policy '{policy_name}' not found. Available: {available}")
    return POLICY_REGISTRY[policy_name]


def list_policies() -> list:
    """Return list of available policy names"""
    return list(POLICY_REGISTRY.keys())
