"""
Policy Inference Module

Python implementation of CudaPolicy for neural network policy inference.
Supports TorchScript models with history buffer.
"""

import torch
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from collections import OrderedDict

from mjsim.policy.policyCfg import load_policy_config, PolicyConfig

# MARK: 环形缓冲区
# 拼接方式 (observation-major, 参考 gpu_multi_circular_buffer.h):
# obs1(history10)  obs1(history9) obs1(history8)  ... obs1(now)
# obs2(history10)  obs2(history9) obs2(history8)  ... obs2(now)
class CircularBuffer:
    """Circular buffer for observation history - observation-major order"""
    
    def __init__(self, obs_specs: List[Tuple[str, int]], history_length: int, device: torch.device, dtype: torch.dtype):
        self.obs_specs = obs_specs
        self.obs_names = [name for name, _ in obs_specs]
        self.history_length = history_length
        self.device = device
        self.dtype = dtype
        
        # Calculate total dimension
        self.total_dim = sum(dim for _, dim in obs_specs)
        
        # Initialize separate buffer for each observation: {name: [history_length, dim]}
        self.buffers: Dict[str, torch.Tensor] = {}
        self.pointers: Dict[str, int] = {}
        
        for name, dim in obs_specs:
            self.buffers[name] = torch.zeros(history_length, dim, dtype=dtype, device=device)
            self.pointers[name] = -1  # -1 means not initialized
    
    def append(self, obs_dict: Dict[str, torch.Tensor]):
        """Append observation to buffer"""
        for name, dim in self.obs_specs:
            if name not in obs_dict:
                continue
            
            current_data = obs_dict[name].view(-1)
            if current_data.device != self.device:
                current_data = current_data.to(self.device)
            
            if self.pointers[name] == -1:
                # First time: fill entire buffer with current data
                for i in range(self.history_length):
                    self.buffers[name][i] = current_data
                self.pointers[name] = 0
            else:
                # Update pointer (circular)
                self.pointers[name] = (self.pointers[name] + 1) % self.history_length
                self.buffers[name][self.pointers[name]] = current_data
    
    def get_stacked_obs(self) -> torch.Tensor:
        """
        Get stacked observations in observation-major order.
        Output: [1, sum(history_length * dim for each obs)]
        
        Format: [obs1_t0..obs1_t9, obs2_t0..obs2_t9, ...]
        """
        flattened_buffers = []
        
        for name, dim in self.obs_specs:
            buffer = self.buffers[name]
            pointer = self.pointers[name]
            
            if pointer == -1:
                # Not initialized, return zeros
                flattened = buffer.flatten()
            else:
                # Roll buffer to get oldest to newest order
                shift = self.history_length - pointer - 1
                ordered_buffer = torch.roll(buffer, shifts=shift, dims=0)
                flattened = ordered_buffer.flatten()
            
            flattened_buffers.append(flattened)
        
        # Concatenate all observations
        stacked = torch.cat(flattened_buffers)
        return stacked.unsqueeze(0)  # [1, total]
    
    def reset(self):
        """Reset buffer to zeros"""
        for name in self.obs_names:
            self.buffers[name].zero_()
            self.pointers[name] = -1


# MARK: 策略
class Policy:
    """
    Neural Network Policy for robot control
    
    Loads TorchScript model and manages observation history for inference.
    """
    
    # Default observation specs: (name, dim, scale)
    DEFAULT_OBS_SPECS = [
        ("base_ang_vel", 3, 0.2),
        ("proj_gravity", 3, 1.0),
        ("vel_command", 3, 1.0),
        ("joint_pos", 20, 1.0),
        ("joint_vel", 20, 0.05),
        ("last_action", 20, 1.0),
        ("gait_phase", 2, 1.0),
    ]
    
    def __init__(
        self,
        model_path: str,
        device: str = "cuda",
        obs_specs: Optional[List[Tuple]] = None,
        obs_scales: Optional[Dict[str, float]] = None,
        history_length: int = 10,
        dtype: torch.dtype = torch.float32
    ):
        """
        Initialize policy
        
        Args:
            model_path: Path to TorchScript model (.pt file)
            device: "cuda" or "cpu"
            obs_specs: List of (name, dim) or (name, dim, scale) tuples
            obs_scales: Dict of observation scales {name: scale} (optional, overrides obs_specs)
            history_length: Number of frames to keep in history buffer
            dtype: Tensor dtype (torch.float32 or torch.float16)
        """
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.dtype = dtype
        self.history_length = history_length
        
        # Parse obs_specs - support both (name, dim) and (name, dim, scale) formats
        if obs_specs is None:
            obs_specs = self.DEFAULT_OBS_SPECS
        
        # Extract (name, dim) and scales
        self.obs_specs = []
        self.obs_scales = {}
        for spec in obs_specs:
            if len(spec) == 2:
                name, dim = spec
                self.obs_specs.append((name, dim))
            elif len(spec) == 3:
                name, dim, scale = spec
                self.obs_specs.append((name, dim))
                self.obs_scales[name] = scale
            else:
                raise ValueError(f"Invalid obs_spec format: {spec}")
        
        # Override with explicit obs_scales if provided
        if obs_scales is not None:
            self.obs_scales.update(obs_scales)
        
        # Initialize observation dictionary
        self.observations: Dict[str, np.ndarray] = {}
        self._initialize_observations()
        
        # Initialize buffer
        self.buffer = CircularBuffer(self.obs_specs, history_length, self.device, dtype)
        
        # Load model
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.module = torch.jit.load(str(model_path), map_location=self.device)
        self.module.to(self.dtype)
        self.module.eval()
        
        # Output tensor
        self.output: Optional[torch.Tensor] = None
        
        # Action dimension (from last_action spec)
        self.action_dim = 20
        for name, dim in self.obs_specs:
            if name == "last_action":
                self.action_dim = dim
                break
        
        self._print_init_info()
    
    def _print_init_info(self):
        """Print colored initialization info"""
        total_dim = sum(dim for _, dim in self.obs_specs)
        print(f"\033[96m{'='*50}\033[0m")
        print(f"\033[96mPolicy Initialized\033[0m")
        print(f"\033[93mModel:\033[0m {self.model_path.name}")
        print(f"\033[93mDevice:\033[0m {self.device}")
        print(f"\033[93mDtype:\033[0m {self.dtype}")
        print(f"\033[93mHistory:\033[0m {self.history_length}")
        print(f"\033[93mObs dim:\033[0m {total_dim} x {self.history_length} = {total_dim * self.history_length}")
        print(f"\033[93mAction dim:\033[0m {self.action_dim}")
        print(f"\033[93mObs keys:\033[0m {list(self.observations.keys())}")
        print(f"\033[96m{'='*50}\033[0m")
    
    def _initialize_observations(self):
        """Initialize observation dictionary with zeros"""
        for name, dim in self.obs_specs:
            self.observations[name] = np.zeros(dim, dtype=np.float64)
    
    def set_observation(self, key: str, value: np.ndarray):
        """Set observation value"""
        if key not in self.observations:
            raise KeyError(f"Observation key '{key}' not found")
        
        expected_dim = len(self.observations[key])
        if len(value) != expected_dim:
            raise ValueError(f"Observation '{key}' size mismatch. Expected: {expected_dim}, got: {len(value)}")
        
        self.observations[key] = np.array(value, dtype=np.float64)
    
    def get_observation(self, key: str) -> np.ndarray:
        """Get observation value"""
        if key not in self.observations:
            raise KeyError(f"Observation key '{key}' not found")
        return self.observations[key]
    
    def obs2buffer(self):
        """Push current observations to buffer with scaling"""
        obs_dict = {}
        for name, dim in self.obs_specs:
            if name in self.observations:
                # Apply scale
                scale = self.obs_scales.get(name, 1.0)
                scaled_obs = self.observations[name] * scale
                tensor = torch.from_numpy(scaled_obs).to(self.dtype).to(self.device)
                obs_dict[name] = tensor
        
        self.buffer.append(obs_dict)
    
    def infer(self) -> torch.Tensor:
        """Run inference and return output tensor"""
        with torch.no_grad():
            stacked_obs = self.buffer.get_stacked_obs()
            self.output = self.module(stacked_obs)
        return self.output
    
    def get_action(self) -> np.ndarray:
        """Get action as numpy array"""
        if self.output is None:
            return np.zeros(self.action_dim)
        return self.output.cpu().numpy().flatten()
    
    def step(self) -> np.ndarray:
        """Convenience method: obs2buffer + infer + get_action"""
        self.obs2buffer()
        self.infer()
        # print(f"{self.model_path}")
        return self.get_action()
    
    # MARK: Warm
    def warm_up(self, iterations: int = 10):
        """Warm up the model with zero observations"""
        action = np.zeros(self.action_dim)
        
        for _ in range(iterations):
            # Reset observations to zero
            for key in self.observations:
                self.observations[key].fill(0.0)
            
            # np.array()[2] = -1.0，设置一整行
            # Set default gravity
            if "projected_gravity" in self.observations:
                self.observations["projected_gravity"][2] = -1.0
            
            # 高度指令必须包含正确指令
            if "height_commands" in self.observations:
                self.observations["height_commands"][0] = 0.665

            # # 默认位置
            # if "joint_pos_rel" in self.observations:
            #     self.controller.set_observation("joint_pos_rel", 
            #         self.q_policy - self.controller.default_pos_policy)
            
            # Set last action
            if "last_action" in self.observations:
                self.observations["last_action"] = action.copy()
            
            self.obs2buffer()
            # print("======="*10)
            # print(self.buffer.get_stacked_obs())
            self.infer()
            action = self.get_action()
        
        print(f"\033[92mPolicy warm up done!\033[0m")
    
    def reset(self):
        """Reset buffer and observations"""
        self.buffer.reset()
        self._initialize_observations()
        self.output = None


def load_policy(policy_name: str, device: str = "cuda") -> Tuple[Policy, PolicyConfig]:
    """
    Load policy from registry
    
    Args:
        policy_name: Policy name in POLICY_REGISTRY
        device: "cuda" or "cpu"
    
    Returns:
        (Policy, PolicyConfig) tuple
    """
    config = load_policy_config(policy_name)
    
    # Find model file
    model_file = config.policy_path / "policy.pt"
    if not model_file.exists():
        # Try other common names
        for name in ["model.pt", "actor.pt", "network.pt"]:
            model_file = config.policy_path / name
            if model_file.exists():
                break
    
    if not model_file.exists():
        raise FileNotFoundError(f"No model file found in {config.policy_path}")
    
    # Build obs_specs from config (now returns (name, dim, scale) tuples)
    obs_specs = config.get_obs_specs()
    
    policy = Policy(
        model_path=str(model_file),
        device=device,
        obs_specs=obs_specs,
        history_length=config.obs_history_length
    )
    
    return policy, config
