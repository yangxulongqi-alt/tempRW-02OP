"""
Motion data reader and Bezier curve interpolator for keyframe-based motion playback.

This module reads motion JSON files containing keyframes with joint positions,
and provides Bezier curve interpolation to calculate joint angles at any given time.
"""

import json
import os
from typing import Dict, List, Optional, Tuple
import numpy as np
from dataclasses import dataclass, field

try:
    from ament_index_python.packages import get_package_share_directory
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


def _get_default_motion_dir() -> str:
    """Get the default motion directory path.
    
    If ROS2 is available, uses package share directory.
    Otherwise falls back to relative path from this file.
    
    Returns:
        Absolute path to the config/motions directory
    """
    if _ROS_AVAILABLE:
        try:
            pkg_share = get_package_share_directory('mjsim')
            return os.path.join(pkg_share, 'config', 'motions')
        except Exception:
            pass
    
    # Fallback to relative path
    return os.path.abspath(os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        '..', 'config', 'motions'
    ))


@dataclass
class Keyframe:
    """Represents a single keyframe with time and joint poses."""
    time: float
    pose: Dict[str, float]

# MARK: Motion
@dataclass
class Motion:
    """Represents a complete motion sequence with metadata and keyframes."""
    id: str
    name: str
    keyframes: List[Keyframe]
    
    @property
    def duration(self) -> float:
        """Get the original duration of the motion (max keyframe time)."""
        if not self.keyframes:
            return 0.0
        return max(kf.time for kf in self.keyframes)
    
    @property
    def joint_names(self) -> List[str]:
        """Get list of all joint names in the motion."""
        if not self.keyframes:
            return []
        return list(self.keyframes[0].pose.keys())

# MARK: Bezier
class BezierInterpolator:
    """
    Bezier curve interpolator for smooth motion transitions.
    
    Uses cubic Bezier curves for interpolation between consecutive keyframes,
    with automatic control point generation for smooth transitions.
    """
    
    def __init__(self, motion: Motion):
        """
        Initialize the Bezier interpolator with a motion sequence.
        
        Args:
            motion: Motion object containing keyframes to interpolate
        """
        self.motion = motion
        self._sorted_keyframes = sorted(motion.keyframes, key=lambda kf: kf.time)
        self._original_duration = motion.duration
        
    @staticmethod
    def _cubic_bezier(t: float, p0: float, p1: float, p2: float, p3: float) -> float:
        """
        Evaluate cubic Bezier curve at parameter t.
        
        Args:
            t: Parameter in [0, 1]
            p0, p1, p2, p3: Control points
            
        Returns:
            Interpolated value at t
        """
        t2 = t * t
        t3 = t2 * t
        mt = 1 - t
        mt2 = mt * mt
        mt3 = mt2 * mt
        
        return mt3 * p0 + 3 * mt2 * t * p1 + 3 * mt * t2 * p2 + t3 * p3
    
    @staticmethod
    def _bezier_n(t: float, points: np.ndarray) -> float:
        """
        Evaluate N-th order Bezier curve using De Casteljau's algorithm.
        
        Args:
            t: Parameter in [0, 1]
            points: Array of control points
            
        Returns:
            Interpolated value at t
        """
        n = len(points)
        if n == 1:
            return points[0]
        
        # De Casteljau's algorithm
        temp = points.copy()
        for r in range(1, n):
            for i in range(n - r):
                temp[i] = (1 - t) * temp[i] + t * temp[i + 1]
        
        return temp[0]
    
    def _generate_control_points(self, joint_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate control points for cubic Bezier curves between keyframes.
        
        Uses Catmull-Rom style tangent estimation for smooth curves.
        
        Args:
            joint_name: Name of the joint
            
        Returns:
            Tuple of (times, values) arrays for all keyframes
        """
        times = np.array([kf.time for kf in self._sorted_keyframes])
        values = np.array([kf.pose.get(joint_name, 0.0) for kf in self._sorted_keyframes])
        return times, values
    
    def get_joint_angle(self, joint_name: str, t: float, T: float) -> float:
        """
        Get interpolated joint angle at time t for total duration T.
        
        Uses cubic Bezier interpolation between keyframes, scaled to fit
        the specified total duration T.
        
        Args:
            joint_name: Name of the joint to interpolate
            t: Current time (should be in [0, T])
            T: Total duration to scale the motion to
            
        Returns:
            Interpolated joint angle at time t
        """
        if not self._sorted_keyframes:
            return 0.0
        
        # Clamp t to valid range
        t = max(0.0, min(t, T))
        
        # Scale t to original motion duration
        if T > 0 and self._original_duration > 0:
            scaled_t = (t / T) * self._original_duration
        else:
            scaled_t = 0.0
        
        times, values = self._generate_control_points(joint_name)
        
        # Handle edge cases
        if len(times) == 1:
            return values[0]
        
        if scaled_t <= times[0]:
            return values[0]
        if scaled_t >= times[-1]:
            return values[-1]
        
        # Find the segment containing scaled_t
        segment_idx = 0
        for i in range(len(times) - 1):
            if times[i] <= scaled_t < times[i + 1]:
                segment_idx = i
                break
        
        # Local parameter within segment [0, 1]
        t0, t1 = times[segment_idx], times[segment_idx + 1]
        local_t = (scaled_t - t0) / (t1 - t0) if t1 != t0 else 0.0
        
        # Get values at segment endpoints
        v0 = values[segment_idx]
        v1 = values[segment_idx + 1]
        
        # Generate control points for cubic Bezier (Catmull-Rom style)
        # Estimate tangents at endpoints
        if segment_idx == 0:
            tangent0 = (v1 - v0) * 0.5
        else:
            tangent0 = (v1 - values[segment_idx - 1]) * 0.25
        
        if segment_idx == len(times) - 2:
            tangent1 = (v1 - v0) * 0.5
        else:
            tangent1 = (values[segment_idx + 2] - v0) * 0.25
        
        # Control points for cubic Bezier
        p0 = v0
        p1 = v0 + tangent0
        p2 = v1 - tangent1
        p3 = v1
        
        return self._cubic_bezier(local_t, p0, p1, p2, p3)
    
    def get_all_joint_angles(self, t: float, T: float) -> Dict[str, float]:
        """
        Get interpolated angles for all joints at time t.
        
        Args:
            t: Current time (should be in [0, T])
            T: Total duration to scale the motion to
            
        Returns:
            Dictionary mapping joint names to their interpolated angles
        """
        result = {}
        for joint_name in self.motion.joint_names:
            result[joint_name] = self.get_joint_angle(joint_name, t, T)
        return result
    
    def get_pose_array(self, t: float, T: float, joint_order: Optional[List[str]] = None) -> np.ndarray:
        """
        Get interpolated joint angles as a numpy array.
        
        Args:
            t: Current time (should be in [0, T])
            T: Total duration to scale the motion to
            joint_order: Optional list specifying the order of joints in output array.
                        If None, uses the order from the motion's first keyframe.
            
        Returns:
            Numpy array of joint angles
        """
        if joint_order is None:
            joint_order = self.motion.joint_names
        
        angles = self.get_all_joint_angles(t, T)
        return np.array([angles.get(name, 0.0) for name in joint_order])

# MARK: Motion reader
class MotionReader:
    """
    Reader for motion JSON files.
    
    Loads and parses motion data from JSON files, providing access to
    individual motions and their Bezier interpolators.
    """
    
    # Default motion directory path (lazy-loaded)
    DEFAULT_MOTION_DIR: str = None  # Will be set on first use
    
    def __init__(self, motion_dir: Optional[str] = None):
        """
        Initialize the motion reader.
        
        Args:
            motion_dir: Directory containing motion JSON files.
                       If None, uses ROS package share directory or fallback path.
        """
        if motion_dir is None:
            self.motion_dir = _get_default_motion_dir()
        else:
            self.motion_dir = os.path.abspath(motion_dir)
        
        self._motions_cache: Dict[str, List[Motion]] = {}
    
    def load_motion_file(self, filename: str) -> List[Motion]:
        """
        Load motions from a JSON file.
        
        Args:
            filename: Name of the JSON file (e.g., 'KungfuSalute2.1.json')
            
        Returns:
            List of Motion objects from the file
            
        Raises:
            FileNotFoundError: If the file doesn't exist
            json.JSONDecodeError: If the file is not valid JSON
        """
        # Check cache first
        if filename in self._motions_cache:
            return self._motions_cache[filename]
        
        filepath = os.path.join(self.motion_dir, filename)
        
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Motion file not found: {filepath}")
        
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        motions = []
        for motion_data in data:
            keyframes = []
            for kf_data in motion_data.get('keyframes', []):
                keyframe = Keyframe(
                    time=float(kf_data.get('time', 0)),
                    pose=kf_data.get('pose', {})
                )
                keyframes.append(keyframe)
            
            motion = Motion(
                id=str(motion_data.get('id', '')),
                name=str(motion_data.get('name', '')),
                keyframes=keyframes
            )
            motions.append(motion)
        
        # Cache the loaded motions
        self._motions_cache[filename] = motions
        
        return motions
    
    def get_motion_by_name(self, filename: str, motion_name: str) -> Optional[Motion]:
        """
        Get a specific motion by name from a file.
        
        Args:
            filename: Name of the JSON file
            motion_name: Name of the motion to find
            
        Returns:
            Motion object if found, None otherwise
        """
        motions = self.load_motion_file(filename)
        for motion in motions:
            if motion.name == motion_name:
                return motion
        return None
    
    def get_motion_by_index(self, filename: str, index: int = 0) -> Optional[Motion]:
        """
        Get a motion by index from a file.
        
        Args:
            filename: Name of the JSON file
            index: Index of the motion (default 0 for first motion)
            
        Returns:
            Motion object if found, None otherwise
        """
        motions = self.load_motion_file(filename)
        if 0 <= index < len(motions):
            return motions[index]
        return None
    
    def create_interpolator(self, filename: str, motion_index: int = 0) -> BezierInterpolator:
        """
        Create a Bezier interpolator for a motion.
        
        Args:
            filename: Name of the JSON file
            motion_index: Index of the motion in the file (default 0)
            
        Returns:
            BezierInterpolator for the specified motion
            
        Raises:
            ValueError: If the motion is not found
        """
        motion = self.get_motion_by_index(filename, motion_index)
        if motion is None:
            raise ValueError(f"Motion not found at index {motion_index} in {filename}")
        return BezierInterpolator(motion)
    
    def list_available_motions(self) -> List[str]:
        """
        List all available motion files in the motion directory.
        
        Returns:
            List of JSON filenames
        """
        if not os.path.exists(self.motion_dir):
            return []
        
        return [f for f in os.listdir(self.motion_dir) if f.endswith('.json')]

# MARK: Motion player
class MotionPlayer:
    """
    High-level motion player that combines reading and interpolation.
    
    Provides a simple interface for playing back motions at specified durations.
    """
    
    def __init__(self, motion_dir: Optional[str] = None):
        """
        Initialize the motion player.
        
        Args:
            motion_dir: Directory containing motion JSON files
        """
        self.reader = MotionReader(motion_dir)
        self._current_interpolator: Optional[BezierInterpolator] = None
        self._current_duration: float = 0.0
    
    def load(self, filename: str, motion_index: int = 0, duration: Optional[float] = None) -> None:
        """
        Load a motion for playback.
        
        Args:
            filename: Name of the JSON file (e.g., 'KungfuSalute2.1.json')
            motion_index: Index of the motion in the file
            duration: Playback duration. If None, uses original motion duration.
        """
        motion = self.reader.get_motion_by_index(filename, motion_index)
        if motion is None:
            raise ValueError(f"Motion not found: {filename}[{motion_index}]")
        
        self._current_interpolator = BezierInterpolator(motion)
        
        if duration is None:
            self._current_duration = motion.duration
        else:
            self._current_duration = duration
    
    def get_pose(self, t: float) -> Dict[str, float]:
        """
        Get the pose at time t during playback.
        
        Args:
            t: Current playback time
            
        Returns:
            Dictionary of joint angles
            
        Raises:
            RuntimeError: If no motion is loaded
        """
        if self._current_interpolator is None:
            raise RuntimeError("No motion loaded. Call load() first.")
        
        return self._current_interpolator.get_all_joint_angles(t, self._current_duration)
    
    def get_pose_array(self, t: float, joint_order: Optional[List[str]] = None) -> np.ndarray:
        """
        Get the pose as a numpy array at time t.
        
        Args:
            t: Current playback time
            joint_order: Optional list specifying joint order
            
        Returns:
            Numpy array of joint angles
        """
        if self._current_interpolator is None:
            raise RuntimeError("No motion loaded. Call load() first.")
        
        return self._current_interpolator.get_pose_array(t, self._current_duration, joint_order)
    
    @property
    def duration(self) -> float:
        """Get the current playback duration."""
        return self._current_duration
    
    @property
    def joint_names(self) -> List[str]:
        """Get list of joint names in the current motion."""
        if self._current_interpolator is None:
            return []
        return self._current_interpolator.motion.joint_names

# MARK: Main
# Example usage and testing
if __name__ == "__main__":
    # Create a motion reader
    reader = MotionReader()
    
    # List available motions
    print("Available motion files:", reader.list_available_motions())
    
    # Load the KungfuSalute motion
    try:
        motion = reader.get_motion_by_index("KungfuSalute2.1.json", 0)
        if motion:
            print(f"\nLoaded motion: {motion.name}")
            print(f"Original duration: {motion.duration}")
            print(f"Number of keyframes: {len(motion.keyframes)}")
            print(f"Joint names: {motion.joint_names}")
            
            # Create interpolator
            interpolator = BezierInterpolator(motion)
            
            # Test with total duration T = 2.0 seconds
            T = 2.0
            print(f"\nInterpolated poses for T = {T} seconds:")
            
            for t in [0.0, 0.5, 1.0, 1.5, 2.0]:
                poses = interpolator.get_all_joint_angles(t, T)
                print(f"\nt = {t}s:")
                for joint, angle in list(poses.items())[:5]:  # Print first 5 joints
                    print(f"  {joint}: {angle:.4f}")
            
            # Using MotionPlayer for simpler interface
            print("\n--- Using MotionPlayer ---")
            player = MotionPlayer()
            player.load("KungfuSalute2.1.json", duration=3.0)
            
            print(f"Playback duration: {player.duration}s")
            pose_at_1s = player.get_pose(1.0)
            print(f"Pose at t=1.0s: {list(pose_at_1s.items())[:3]}...")
            
    except FileNotFoundError as e:
        print(f"Error: {e}")
