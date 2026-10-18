"""
Data Logger Node

A ROS2 node for logging JointState and Imu data to pickle files.
Creates log files organized by model type and timestamp.

Usage:
    ros2 run mjsim log_node --ros-args -p model_type:=honghu
    
Log structure:
    LOG_DIR/
    └── {model_type}/
        └── {YYYYMMDD_HHMMSS}.pkl
"""

import rclpy
from rclpy.node import Node
import numpy as np
import pickle
import os
from datetime import datetime
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field

from sensor_msgs.msg import JointState, Imu

from mjsim.cfgs import LOG_DIR


@dataclass
class LogEntry:
    """Single log entry containing timestamp and sensor data."""
    timestamp: float
    joint_state: Optional[Dict[str, Any]] = None
    imu: Optional[Dict[str, Any]] = None


class LogNode(Node):
    """
    ROS2 node for logging JointState and Imu data.
    
    Parameters:
        model_type (str): Model type name, used as subdirectory name
        log_rate (int): Logging rate in Hz (default: 50)
        max_entries (int): Maximum entries before auto-save (default: 10000)
    """
    
    def __init__(self):
        super().__init__('log_node')
        
        # Declare parameters
        self.declare_parameter('model_type', 'default')
        self.declare_parameter('log_rate', 50)
        self.declare_parameter('max_entries', 10000)
        
        # Get parameters
        self.model_type = self.get_parameter('model_type').get_parameter_value().string_value
        self.log_rate = self.get_parameter('log_rate').get_parameter_value().integer_value
        self.max_entries = self.get_parameter('max_entries').get_parameter_value().integer_value
        
        # Create log directory
        self.log_dir = LOG_DIR / self.model_type
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Create log file with timestamp
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = self.log_dir / f'{timestamp_str}.pkl'
        
        # Data buffers
        self.log_entries: List[LogEntry] = []
        self.current_joint_state: Optional[JointState] = None
        self.current_imu: Optional[Imu] = None
        
        # Statistics
        self.joint_state_count = 0
        self.imu_count = 0
        self.log_count = 0
        
        # Set up subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            10
        )
        
        # Logging timer
        log_period = 1.0 / self.log_rate
        self.log_timer = self.create_timer(log_period, self._log_callback)
        
        # Status print timer (every 5 seconds)
        self.status_timer = self.create_timer(5.0, self._print_status)
        
        # Print initialization info
        self._print_init_info()
    
    def _joint_state_callback(self, msg: JointState):
        """Store latest JointState message."""
        self.current_joint_state = msg
        self.joint_state_count += 1
    
    def _imu_callback(self, msg: Imu):
        """Store latest Imu message."""
        self.current_imu = msg
        self.imu_count += 1
    
    def _log_callback(self):
        """Log current sensor data at fixed rate."""
        entry = LogEntry(
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
        # Convert JointState to dict
        if self.current_joint_state is not None:
            js = self.current_joint_state
            entry.joint_state = {
                'name': list(js.name),
                'position': list(js.position),
                'velocity': list(js.velocity),
                'effort': list(js.effort) if js.effort else []
            }
        
        # Convert Imu to dict
        if self.current_imu is not None:
            imu = self.current_imu
            entry.imu = {
                'orientation': {
                    'x': imu.orientation.x,
                    'y': imu.orientation.y,
                    'z': imu.orientation.z,
                    'w': imu.orientation.w
                },
                'angular_velocity': {
                    'x': imu.angular_velocity.x,
                    'y': imu.angular_velocity.y,
                    'z': imu.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': imu.linear_acceleration.x,
                    'y': imu.linear_acceleration.y,
                    'z': imu.linear_acceleration.z
                }
            }
        
        self.log_entries.append(entry)
        self.log_count += 1
        
        # Auto-save when reaching max entries
        if len(self.log_entries) >= self.max_entries:
            self.save_log()
            self.log_entries = []
            self.get_logger().info(f'\033[93m[AutoSave] Saved {self.max_entries} entries, starting new buffer\033[0m')
    
    def save_log(self):
        """Save log entries to pickle file."""
        if not self.log_entries:
            self.get_logger().warn('No entries to save')
            return
        
        # Prepare data for saving
        data = {
            'model_type': self.model_type,
            'log_rate': self.log_rate,
            'num_entries': len(self.log_entries),
            'entries': self.log_entries
        }
        
        # Save to pickle file
        with open(self.log_file, 'wb') as f:
            pickle.dump(data, f)
        
        self.get_logger().info(f'\033[92m[Saved] {len(self.log_entries)} entries to {self.log_file}\033[0m')
    
    def _print_status(self):
        """Print periodic status."""
        self.get_logger().info(
            f'\033[94m[Status]\033[0m Logged: {self.log_count}, '
            f'JointState msgs: {self.joint_state_count}, '
            f'Imu msgs: {self.imu_count}, '
            f'Buffer: {len(self.log_entries)}/{self.max_entries}'
        )
    
    def _print_init_info(self):
        """Print initialization information."""
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
        self.get_logger().info(f'\033[96mLog Node Started\033[0m')
        self.get_logger().info(f'\033[93mModel type:\033[0m {self.model_type}')
        self.get_logger().info(f'\033[93mLog rate:\033[0m {self.log_rate} Hz')
        self.get_logger().info(f'\033[93mMax entries:\033[0m {self.max_entries}')
        self.get_logger().info(f'\033[93mLog directory:\033[0m {self.log_dir}')
        self.get_logger().info(f'\033[93mLog file:\033[0m {self.log_file}')
        self.get_logger().info(f'\033[93mTopics:\033[0m')
        self.get_logger().info(f'  Sub: /joint_states, /imu/data')
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
    
    def destroy_node(self):
        """Save remaining log entries before shutdown."""
        if self.log_entries:
            self.get_logger().info(f'\033[93m[Shutdown] Saving remaining {len(self.log_entries)} entries...\033[0m')
            self.save_log()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = LogNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
