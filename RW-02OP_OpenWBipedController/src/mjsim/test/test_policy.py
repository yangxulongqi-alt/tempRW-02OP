"""
Policy Test

Load data.json and compare policy inference with recorded actions frame by frame.
Reference: rl_honghu/test/test_rl_policy_honghu.cpp
"""

import json
import numpy as np
from pathlib import Path
import sys
from dataclasses import dataclass
from typing import Optional

# Add package path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mjsim.policy.policy import Policy
from mjsim.policy.policyCfg import load_policy_config
from mjsim.cfgs import POLICY_REGISTRY


# Color codes
C_R = '\033[91m'  # Red
C_G = '\033[92m'  # Green
C_Y = '\033[93m'  # Yellow
C_C = '\033[96m'  # Cyan
C_End = '\033[0m'


# MARK: Utils
def load_json_data(policy_name: str) -> list:
    """Load data.json from policy directory"""
    data_path = POLICY_REGISTRY[policy_name] / 'data.json'
    print(f"{C_G}Loading data from: {data_path}{C_End}")
    
    with open(data_path, 'r') as f:
        data = json.load(f)
    
    print(f"{C_G}Loaded {len(data)} frames{C_End}")
    return data


def create_policy(policy_name: str, device: str = 'cuda') -> tuple:
    """Create policy and return (policy, policy_cfg)"""
    policy_cfg = load_policy_config(policy_name)
    policy_path = POLICY_REGISTRY[policy_name] / 'policy.pt'
    obs_specs = policy_cfg.get_obs_specs()
    
    policy = Policy(
        model_path=str(policy_path),
        device=device,
        obs_specs=obs_specs,
        history_length=policy_cfg.obs_history_length
    )
    return policy, policy_cfg


@dataclass
class FrameObs:
    """Observation data for a single frame"""
    base_ang_vel: np.ndarray
    proj_gravity: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    cmd_vel: np.ndarray
    last_action: np.ndarray
    gait_phase: np.ndarray


def load_frame_obs(obs_frame: dict, robot_id: int) -> FrameObs:
    """Load observations from a data frame"""
    return FrameObs(
        base_ang_vel=np.array(obs_frame["base_ang_vel"][robot_id]),
        proj_gravity=np.array(obs_frame["proj_gravty"][robot_id]),  # Note: typo in data.json
        joint_pos=np.array(obs_frame["joint_pos"][robot_id]),
        joint_vel=np.array(obs_frame["joint_vel"][robot_id]),
        cmd_vel=np.array(obs_frame["cmd_vel"][robot_id]),
        last_action=np.array(obs_frame["last_action"][robot_id]),
        gait_phase=np.array(obs_frame["gait_phase"][robot_id]),
    )


def set_policy_obs(policy: Policy, obs: FrameObs):
    """Set policy observations from FrameObs"""
    policy.set_observation("base_ang_vel", obs.base_ang_vel)
    policy.set_observation("projected_gravity", obs.proj_gravity)
    policy.set_observation("joint_pos_rel", obs.joint_pos)
    policy.set_observation("joint_vel_rel", obs.joint_vel)
    policy.set_observation("velocity_commands", obs.cmd_vel)
    policy.set_observation("last_action", obs.last_action)
    policy.set_observation("gait_phase", obs.gait_phase)


def print_frame_obs(obs: FrameObs, frame_idx: int):
    """Print loaded observations"""
    np.set_printoptions(precision=3, suppress=True)
    print(f"{C_Y}---------------- Frame {frame_idx:3d} ----------------{C_End}")
    print(f"{C_C}Loaded Observations:{C_End}")
    print(f"  base_ang_vel     : {obs.base_ang_vel}")
    print(f"  proj_gravity     : {obs.proj_gravity}")
    print(f"  velocity_commands: {obs.cmd_vel}")
    print(f"  joint_pos_rel    : {obs.joint_pos[:5]}...")
    print(f"  joint_vel_rel    : {obs.joint_vel[:5]}...")
    print(f"  last_action      : {obs.last_action[:5]}...")
    print(f"  gait_phase       : {obs.gait_phase}")


def print_summary(title: str, num_frames: int, max_diff: float, mean_diff: float, err_cnt: int):
    """Print test summary"""
    print(f"{C_C}{'='*60}{C_End}")
    print(f"{C_Y}{title}:{C_End}")
    print(f"  Total frames: {num_frames}")
    print(f"  Max diff: {max_diff:.3f}")
    print(f"  Mean diff: {mean_diff:.3f}")
    print(f"  Elements exceeding 1e-6: {err_cnt}")


# MARK: 观测组装测试
def test_obs_buffer(policy_name: str = 'panda_walk', num_frames: int = 100, 
                    robot_id: int = 0, target_frame: int = -1):
    """Compare assembled observation buffer with recorded obs_input"""
    json_data = load_json_data(policy_name)
    num_frames = min(num_frames, len(json_data))
    
    policy, policy_cfg = create_policy(policy_name)
    policy.reset()
    
    print(f"{C_C}{'='*60}{C_End}")
    print(f"{C_C}Observation Buffer Test: {policy_name}{C_End}")
    print(f"{C_C}{'='*60}{C_End}")
    
    max_obs_diff = 0.0
    total_obs_diff = 0.0
    err_exceed_cnt = 0
    
    for i in range(num_frames):
        obs = load_frame_obs(json_data[i]['obs_frame'], robot_id)
        set_policy_obs(policy, obs)
        policy.obs2buffer()
        
        # Get assembled vs expected
        obs_assembled = policy.buffer.get_stacked_obs().cpu().numpy().flatten()
        obs_expected = np.array(json_data[i]['obs_input'][robot_id])
        
        # Compute difference
        min_len = min(len(obs_assembled), len(obs_expected))
        obs_diff = obs_assembled[:min_len] - obs_expected[:min_len]
        
        frame_max_diff = np.max(np.abs(obs_diff))
        frame_mean_diff = np.mean(np.abs(obs_diff))
        max_obs_diff = max(max_obs_diff, frame_max_diff)
        total_obs_diff += frame_mean_diff
        err_exceed_cnt += np.sum(np.abs(obs_diff) > 1e-6)
        
        # Print output
        if target_frame >= 0:
            if i == target_frame:
                print_frame_obs(obs, i)
                np.set_printoptions(precision=3, suppress=True)
                print(f"Obs len: assembled={len(obs_assembled)}, expected={len(obs_expected)}")
                print(f"{C_C}Assembled:{C_End}\n{obs_assembled}")
                print(f"{C_C}Expected:{C_End}\n{obs_expected}")
                print(f"{C_C}Diff:{C_End}\n{obs_diff}")
                print(f"{C_Y}Max diff: {frame_max_diff:.3f}, Mean diff: {frame_mean_diff:.3f}{C_End}")
        else:
            print(f"{C_Y}Frame {i:3d}{C_End} | Max diff: {frame_max_diff:.3f}")
    
    print_summary("Observation Buffer Test Summary", num_frames, max_obs_diff, 
                  total_obs_diff / num_frames, err_exceed_cnt)
    return err_exceed_cnt == 0


# MARK: 动作输出测试
def test_policy_output(policy_name: str = 'panda_walk', num_frames: int = 100, robot_id: int = 0):
    """Test policy output against recorded data"""
    json_data = load_json_data(policy_name)
    num_frames = min(num_frames, len(json_data))
    
    policy, policy_cfg = create_policy(policy_name)
    policy.warm_up()
    
    print(f"{C_C}{'='*60}{C_End}")
    print(f"{C_C}Action Output Test: {policy_name}{C_End}")
    print(f"{C_C}{'='*60}{C_End}")
    
    max_diff = 0.0
    total_diff = 0.0
    err_exceed_cnt = 0
    
    for i in range(num_frames):
        obs = load_frame_obs(json_data[i]['obs_frame'], robot_id)
        set_policy_obs(policy, obs)
        print_frame_obs(obs, i)
        
        # Get action
        action_out = policy.step()
        action_expected = np.array(json_data[i]['action'][robot_id])
        
        # Compute difference
        action_diff = action_out - action_expected
        frame_max_diff = np.max(np.abs(action_diff))
        frame_mean_diff = np.mean(np.abs(action_diff))
        max_diff = max(max_diff, frame_max_diff)
        total_diff += frame_mean_diff
        err_exceed_cnt += np.sum(np.abs(action_diff) > 1e-6)
        
        # Print action comparison
        print(f"{C_C}Action Comparison:{C_End}")
        print(f"  Out : {action_out[:5]}...")
        print(f"  Load: {action_expected[:5]}...")
        
        diff_str = "  Diff: ["
        for j, d in enumerate(action_diff):
            color = C_R if abs(d) > 1e-6 else C_G
            diff_str += f"{color}{d:.3f}{C_End}"
            if j < len(action_diff) - 1:
                diff_str += ", "
        print(diff_str + "]")
    
    print_summary("Action Output Test Summary", num_frames, max_diff, 
                  total_diff / num_frames, err_exceed_cnt)
    return err_exceed_cnt == 0


# MARK: Main
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Test policy against recorded data')
    parser.add_argument('--policy', type=str, default='panda_walk', help='Policy name')
    parser.add_argument('--frames', type=int, default=100, help='Number of frames to test')
    parser.add_argument('--robot_id', type=int, default=0, help='Robot ID (0 or 1)')
    parser.add_argument('--test', type=str, default='obs', choices=['obs', 'action', 'all'],
                        help='Test type: obs (buffer), action, or all')
    parser.add_argument('--frame', type=int, default=-1, help='Target frame for full output')
    args = parser.parse_args()
    
    if args.test == 'obs':
        success = test_obs_buffer(args.policy, args.frames, args.robot_id, args.frame)
    elif args.test == 'action':
        success = test_policy_output(args.policy, args.frames, args.robot_id)
    else:
        obs_ok = test_obs_buffer(args.policy, args.frames, args.robot_id)
        action_ok = test_policy_output(args.policy, args.frames, args.robot_id)
        success = obs_ok and action_ok
    
    print(f"\n{C_G if success else C_Y}{'All tests passed!' if success else 'Test completed with differences'}{C_End}")


if __name__ == '__main__':
    main()
