# Generic MuJoCo Simulation Manager
"""
MJCF模型文件要求
=========================

1. Actuators:
   - 每个可控制的关节必须有一个驱动器定义
   - 驱动器名称将被用于关节名称在JointState msg中
   - Example:
     <actuator>
       <motor name="joint1" joint="joint1" .../>
       <motor name="joint2" joint="joint2" .../>
     </actuator>

2. IMU Sensors (optional, auto-discovered by name pattern):
   - Orientation (framequat): names containing 'orientation', 'framequat', 'quat', 'imu_quat'
   - Gyroscope: names containing 'angular-velocity', 'gyro', 'angular_velocity', 'imu_gyro'
   - Accelerometer: names containing 'linear-acceleration', 'accelerometer', 'linear_acceleration', 'imu_acc'
   - Example:
     <sensor>
       <framequat name="orientation" objtype="site" objname="imu_site"/>
       <gyro name="angular-velocity" site="imu_site"/>
       <accelerometer name="linear-acceleration" site="imu_site"/>
     </sensor>

3. IMU Site:
   - 在安装IMU的机身上创建一个site
   - Example:
     <body name="base_link">
       <site name="imu_site" pos="0 0 0"/>
     </body>
"""

import mujoco
import mujoco.viewer as viewer
import numpy as np
import time
import os
import sys

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class MITCommand:
    """MIT mode control command for a single joint"""
    def __init__(self):
        self.tau_ff = 0.0    # feedforward torque
        self.pos_des = 0.0   # desired position
        self.vel_des = 0.0   # desired velocity
        self.kp = 0.0        # position gain
        self.kd = 0.0        # velocity gain

# MARK: Manager
class SimManager:
    """
    Generic MuJoCo Simulation Manager
    
    Features:
    - Auto-discovers actuators and joints from XML
    - MIT-style motor control interface
    - JointState and IMU data acquisition
    - Optional viewer support
    """
    
    def __init__(self, xml_path: str, enable_viewer: bool = True, viewer_sync_interval: int = 30):
        """
        Initialize simulation manager
        
        Args:
            xml_path: Path to MuJoCo XML model file
            enable_viewer: Whether to launch passive viewer
            viewer_sync_interval: Sync viewer every N frames (0 = every frame)
        """
        if not os.path.exists(xml_path):
            print(f"\033[91mError: XML file not found: {xml_path}\033[0m")
            sys.exit(1)
        
        # Load model and create data
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # Discover actuators (controllable joints)
        self.actuator_num = self.model.nu
        self.actuator_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator_{i}"
            for i in range(self.actuator_num)
        ]
        
        # Build actuator -> joint mapping
        self._actuator_jnt_qposadr = np.zeros(self.actuator_num, dtype=np.int32)
        self._actuator_jnt_dofadr = np.zeros(self.actuator_num, dtype=np.int32)
        for i in range(self.actuator_num):
            trnid = self.model.actuator_trnid[i, 0]
            self._actuator_jnt_qposadr[i] = self.model.jnt_qposadr[trnid]
            self._actuator_jnt_dofadr[i] = self.model.jnt_dofadr[trnid]
        
        # Discover IMU sensors
        self._imu_sensors = self._discover_imu_sensors()
        
        # Timing
        self.dt = self.model.opt.timestep
        self.fps = 1.0 / self.dt
        self.sim_time = 0.0
        self.frame_count = 0
        
        # Control buffer
        self._mit_commands = [MITCommand() for _ in range(self.actuator_num)]
        self._control_mode = 'torque'  # 'torque' or 'mit'
        
        # Viewer
        self.viewer = None
        self._viewer_sync_interval = viewer_sync_interval
        if enable_viewer:
            self.viewer = viewer.launch_passive(
                self.model, self.data,
                show_left_ui=False, show_right_ui=False
            )
            self.viewer.cam.distance = 3.0
            self.viewer.cam.elevation = -20
            self.viewer.cam.azimuth = 135
        
        # Camera tracking
        self._camera_track_body = None  # Body name to track (None = disabled)
        self._camera_track_body_id = -1
        self._camera_track_distance = 3.0  # Distance from tracked body
        self._camera_track_filter_xy = 0.1  # Low-pass filter alpha for XY (0~1, lower = smoother)
        self._camera_track_filter_z = 0.02  # Low-pass filter alpha for Z (stronger filtering)
        self._camera_lookat_filtered = None  # Filtered lookat position [3]
        
        # Print init info
        self._print_init_info(xml_path)
    
    def _print_init_info(self, xml_path: str):
        """Print colored initialization info"""
        print(f"\033[96m{'='*50}\033[0m")
        print(f"\033[96mSimManager Initialized\033[0m")
        print(f"\033[93mModel:\033[0m {os.path.basename(xml_path)}")
        print(f"\033[93mActuators:\033[0m {self.actuator_num}")
        for i, name in enumerate(self.actuator_names):
            jnt_id = self.model.actuator_trnid[i, 0]
            axis = self.model.jnt_axis[jnt_id]
            axis_sign = int(np.sign(axis[np.argmax(np.abs(axis))]))
            print(f"  [{i:2d}] {name} (dir: {axis_sign:+d})")
        print(f"\033[93mTimestep:\033[0m {self.dt*1000:.2f} ms ({self.fps:.0f} Hz)")
        print(f"\033[93mIMU Sensors:\033[0m", end=" ")
        if self._imu_sensors['orientation'] is not None:
            print("orientation", end=" ")
        if self._imu_sensors['gyro'] is not None:
            print("gyro", end=" ")
        if self._imu_sensors['acc'] is not None:
            print("accelerometer", end=" ")
        if all(v is None for v in self._imu_sensors.values()):
            print("(none found)")
        else:
            print()
        print(f"\033[96m{'='*50}\033[0m")
    
    def _discover_imu_sensors(self) -> dict:
        """Auto-discover IMU-related sensors"""
        sensors = {'orientation': None, 'gyro': None, 'acc': None}
        
        # Common sensor name patterns
        orient_names = ['orientation', 'framequat', 'quat', 'imu_quat', 'base_orientation']
        gyro_names = ['angular-velocity', 'gyro', 'angular_velocity', 'imu_gyro', 'base_gyro']
        acc_names = ['linear-acceleration', 'accelerometer', 'linear_acceleration', 'imu_acc', 'base_acc']
        
        for i in range(self.model.nsensor):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            if name is None:
                continue
            name_lower = name.lower()
            
            for pattern in orient_names:
                if pattern in name_lower and sensors['orientation'] is None:
                    sensors['orientation'] = i
                    break
            for pattern in gyro_names:
                if pattern in name_lower and sensors['gyro'] is None:
                    sensors['gyro'] = i
                    break
            for pattern in acc_names:
                if pattern in name_lower and sensors['acc'] is None:
                    sensors['acc'] = i
                    break
        
        return sensors
    
    # ==================== Observation ====================
    # MARK: 观测
    
    def _get_ros_time(self) -> Time:
        """Get current time as ROS Time message"""
        now_ns = time.time_ns()
        t = Time()
        t.sec = int(now_ns // 1_000_000_000)
        t.nanosec = int(now_ns % 1_000_000_000)
        return t
    
    def get_joint_state(self) -> JointState:
        """Get current joint states for all actuated joints (ROS JointState msg)"""
        js = JointState()
        js.header = Header()
        js.header.stamp = self._get_ros_time()
        js.header.frame_id = 'world'
        
        js.name = self.actuator_names
        js.position = [0.0] * self.actuator_num
        js.velocity = [0.0] * self.actuator_num
        js.effort = [0.0] * self.actuator_num
        
        for i in range(self.actuator_num):
            qpos_adr = self._actuator_jnt_qposadr[i]
            dof_adr = self._actuator_jnt_dofadr[i]
            
            js.position[i] = float(self.data.qpos[qpos_adr])
            js.velocity[i] = float(self.data.qvel[dof_adr])
            js.effort[i] = float(self.data.actuator_force[i])
        
        return js
    
    def get_imu_data(self) -> Imu:
        """Get IMU sensor data (ROS Imu msg)"""
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = self._get_ros_time()
        imu.header.frame_id = 'imu_link'
        
        # Orientation (quaternion) - ROS uses x,y,z,w order
        if self._imu_sensors['orientation'] is not None:
            sensor_id = self._imu_sensors['orientation']
            adr = self.model.sensor_adr[sensor_id]
            # MuJoCo: w,x,y,z -> ROS: x,y,z,w
            imu.orientation.w = float(self.data.sensordata[adr + 0])
            imu.orientation.x = float(self.data.sensordata[adr + 1])
            imu.orientation.y = float(self.data.sensordata[adr + 2])
            imu.orientation.z = float(self.data.sensordata[adr + 3])
        else:
            imu.orientation.w = 1.0
        
        # Angular velocity (gyroscope)
        if self._imu_sensors['gyro'] is not None:
            sensor_id = self._imu_sensors['gyro']
            adr = self.model.sensor_adr[sensor_id]
            imu.angular_velocity.x = float(self.data.sensordata[adr + 0])
            imu.angular_velocity.y = float(self.data.sensordata[adr + 1])
            imu.angular_velocity.z = float(self.data.sensordata[adr + 2])
        
        # Linear acceleration
        if self._imu_sensors['acc'] is not None:
            sensor_id = self._imu_sensors['acc']
            adr = self.model.sensor_adr[sensor_id]
            imu.linear_acceleration.x = float(self.data.sensordata[adr + 0])
            imu.linear_acceleration.y = float(self.data.sensordata[adr + 1])
            imu.linear_acceleration.z = float(self.data.sensordata[adr + 2])
        
        return imu
    
    def get_body_pose(self, body_name: str) -> tuple:
        """
        Get body position and orientation
        
        Returns:
            (pos, quat): position [3] and quaternion [4] (w,x,y,z)
        """
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return None, None
        pos = self.data.xpos[body_id].copy()
        quat = self.data.xquat[body_id].copy()
        return pos, quat
    
    def get_joint_limits(self) -> dict:
        """
        Get joint position limits for all actuated joints
        
        Returns:
            dict: {
                'names': [joint_name, ...],
                'lower': [lower_limit, ...],
                'upper': [upper_limit, ...]
            }
        """
        names = []
        lower = []
        upper = []
        
        for i in range(self.actuator_num):
            # Get the joint id associated with this actuator
            jnt_id = self.model.actuator_trnid[i, 0]
            
            # Check if joint has limits (limited=1 in mjcf)
            if self.model.jnt_limited[jnt_id]:
                jnt_range = self.model.jnt_range[jnt_id]
                lower_val = float(jnt_range[0])
                upper_val = float(jnt_range[1])
            else:
                # No limits defined, use +/- infinity
                lower_val = float('-inf')
                upper_val = float('inf')
            
            names.append(self.actuator_names[i])
            lower.append(lower_val)
            upper.append(upper_val)
        
        return {
            'names': names,
            'lower': lower,
            'upper': upper
        }
    
    def get_center_of_mass(self, body_name: str = None) -> np.ndarray:
        """
        获取模型质心所在的空间点坐标
        
        Args:
            body_name: If specified, returns CoM of the subtree rooted at this body.
                       If None, returns the overall CoM of the entire model (subtree of world).
        
        Returns:
            np.ndarray: Center of mass position [x, y, z] in world frame.
                        Returns None if body_name is invalid.
        
        Note:
            MuJoCo computes subtree_com for each body, which is the CoM of all bodies
            in the subtree rooted at that body. The world body (index 0) gives the
            overall model CoM.
        """
        if body_name is None:
            # Return overall CoM (subtree of world body, index 0)
            return self.data.subtree_com[0].copy()
        else:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0:
                print(f"\033[91mError: Body '{body_name}' not found\033[0m")
                return None
            return self.data.subtree_com[body_id].copy()
    
    # ==================== Control ====================
    # MARK: 控制
    
    def set_torque(self, tau: np.ndarray):
        """
        Apply torque control to all joints
        
        Args:
            tau: Torque array [actuator_num]
        """
        self._control_mode = 'torque'
        n = min(len(tau), self.actuator_num)
        self.data.ctrl[:n] = tau[:n]
    
    def set_mit_command(self, idx: int, tau_ff: float, pos_des: float, vel_des: float, kp: float, kd: float):
        """
        Set MIT-style command for a single joint
        
        Args:
            idx: Joint index
            tau_ff: Feedforward torque
            pos_des: Desired position
            vel_des: Desired velocity
            kp: Position gain
            kd: Velocity gain
        """
        if 0 <= idx < self.actuator_num:
            self._control_mode = 'mit'
            cmd = self._mit_commands[idx]
            cmd.tau_ff = tau_ff
            cmd.pos_des = pos_des
            cmd.vel_des = vel_des
            cmd.kp = kp
            cmd.kd = kd
    
    def set_mit_commands(self, tau_ff: np.ndarray, pos_des: np.ndarray, vel_des: np.ndarray,
                         kp: np.ndarray, kd: np.ndarray):
        """
        Set MIT-style commands for all joints
        
        Args:
            tau_ff: Feedforward torque [actuator_num]
            pos_des: Desired position [actuator_num]
            vel_des: Desired velocity [actuator_num]
            kp: Position gains [actuator_num]
            kd: Velocity gains [actuator_num]
        """
        self._control_mode = 'mit'
        for i in range(self.actuator_num):
            cmd = self._mit_commands[i]
            cmd.tau_ff = tau_ff[i] if i < len(tau_ff) else 0.0
            cmd.pos_des = pos_des[i] if i < len(pos_des) else 0.0
            cmd.vel_des = vel_des[i] if i < len(vel_des) else 0.0
            cmd.kp = kp[i] if i < len(kp) else 0.0
            cmd.kd = kd[i] if i < len(kd) else 0.0
    
    def set_mit_flat(self, data: np.ndarray):
        """
        Set MIT commands from flat array [tau_ff, pos_des, vel_des, kp, kd]
        
        Args:
            data: Flat array of size 5 * actuator_num
        """
        n = self.actuator_num
        if len(data) != 5 * n:
            print(f"\033[91mError: Expected {5*n} values, got {len(data)}\033[0m")
            return
        
        self.set_mit_commands(
            tau_ff=data[0:n],
            pos_des=data[n:2*n],
            vel_des=data[2*n:3*n],
            kp=data[3*n:4*n],
            kd=data[4*n:5*n]
        )
    
    def set_hold_position(self, kp=30.0, kd=1.0, pos_des: np.ndarray = None):
        """
        Set MIT PD control to hold position (for initialization)
        
        Args:
            kp: Position gain (scalar or array [actuator_num])
            kd: Velocity gain (scalar or array [actuator_num])
            pos_des: Desired position array. If None, uses zeros (hold at zero)
        """
        n = self.actuator_num
        tau_ff = np.zeros(n)
        vel_des = np.zeros(n)
        
        # Handle kp (scalar or array)
        if np.isscalar(kp):
            kp_arr = np.full(n, kp)
        else:
            kp_arr = np.array(kp)
            if len(kp_arr) != n:
                kp_arr = np.pad(kp_arr, (0, max(0, n - len(kp_arr))), constant_values=kp_arr[-1])[:n]
        
        # Handle kd (scalar or array)
        if np.isscalar(kd):
            kd_arr = np.full(n, kd)
        else:
            kd_arr = np.array(kd)
            if len(kd_arr) != n:
                kd_arr = np.pad(kd_arr, (0, max(0, n - len(kd_arr))), constant_values=kd_arr[-1])[:n]
        
        # Handle pos_des
        if pos_des is None:
            pos_des = np.zeros(n)
        else:
            pos_des = np.array(pos_des)
            if len(pos_des) != n:
                pos_des = np.pad(pos_des, (0, max(0, n - len(pos_des))))[:n]
        
        self.set_mit_commands(tau_ff, pos_des, vel_des, kp_arr, kd_arr)
        print(f"\033[92mHold position set: kp={kp_arr[:3]}..., kd={kd_arr[:3]}..., pos={pos_des[:3]}...\033[0m")
    
    def _apply_mit_control(self):
        """Compute and apply MIT PD control"""
        js = self.get_joint_state()
        
        for i in range(self.actuator_num):
            cmd = self._mit_commands[i]
            pos_err = cmd.pos_des - js.position[i]
            vel_err = cmd.vel_des - js.velocity[i]
            tau = cmd.tau_ff + cmd.kp * pos_err + cmd.kd * vel_err
            self.data.ctrl[i] = tau
    
    # ==================== Simulation ====================
    # MARK: Step
    
    def step(self):
        """Advance simulation by one timestep"""
        # Apply control
        if self._control_mode == 'mit':
            self._apply_mit_control()
        
        # Step physics
        mujoco.mj_step(self.model, self.data)
        
        self.frame_count += 1
        self.sim_time = self.frame_count * self.dt
        
        # Sync viewer
        if self.viewer is not None:
            if self._viewer_sync_interval == 0 or self.frame_count % self._viewer_sync_interval == 0:
                self._update_camera_tracking()
                self.viewer.sync()
    
    def reset(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        self.frame_count = 0
        self.sim_time = 0.0
        
        # Reset control commands
        for cmd in self._mit_commands:
            cmd.tau_ff = 0.0
            cmd.pos_des = 0.0
            cmd.vel_des = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
        self._control_mode = 'torque'
    
    def set_joint_positions(self, positions: np.ndarray, velocities: np.ndarray = None):
        """
        Set joint positions (and optionally velocities) directly
        
        Args:
            positions: Position array [actuator_num]
            velocities: Velocity array [actuator_num], defaults to zeros
        """
        if velocities is None:
            velocities = np.zeros(self.actuator_num)
        
        for i in range(min(len(positions), self.actuator_num)):
            qpos_adr = self._actuator_jnt_qposadr[i]
            dof_adr = self._actuator_jnt_dofadr[i]
            self.data.qpos[qpos_adr] = positions[i]
            self.data.qvel[dof_adr] = velocities[i] if i < len(velocities) else 0.0
        
        mujoco.mj_forward(self.model, self.data)
    
    def set_timestep(self, dt: float):
        """Set simulation timestep"""
        self.model.opt.timestep = dt
        self.dt = dt
        self.fps = 1.0 / dt
    
    # ==================== Viewer ====================
    
    def set_camera(self, distance: float = None, elevation: float = None, 
                   azimuth: float = None, lookat: np.ndarray = None):
        """Configure viewer camera"""
        if self.viewer is None:
            return
        if distance is not None:
            self.viewer.cam.distance = distance
        if elevation is not None:
            self.viewer.cam.elevation = elevation
        if azimuth is not None:
            self.viewer.cam.azimuth = azimuth
        if lookat is not None:
            self.viewer.cam.lookat[:] = lookat
    
    def enable_camera_tracking(self, body_name: str, distance: float = 3.0,
                                filter_xy: float = 0.1, filter_z: float = 0.02):
        """
        Enable camera tracking for a specific body
        
        The camera will follow the target body while keeping its attitude
        (elevation and azimuth) unchanged. Uses low-pass filtering to smooth
        camera motion, with stronger filtering on Z axis to reduce walking jitter.
        
        Args:
            body_name: Name of the body to track
            distance: Camera distance from the tracked body
            filter_xy: Low-pass filter alpha for XY axes (0~1, lower = smoother)
            filter_z: Low-pass filter alpha for Z axis (default more aggressive)
        """
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            print(f"\033[91mError: Body '{body_name}' not found for camera tracking\033[0m")
            return False
        
        self._camera_track_body = body_name
        self._camera_track_body_id = body_id
        self._camera_track_distance = distance
        self._camera_track_filter_xy = np.clip(filter_xy, 0.001, 1.0)
        self._camera_track_filter_z = np.clip(filter_z, 0.001, 1.0)
        
        # Initialize filtered position to current body position
        self._camera_lookat_filtered = self.data.xpos[body_id].copy()
        
        if self.viewer is not None:
            self.viewer.cam.distance = distance
            self.viewer.cam.lookat[:] = self._camera_lookat_filtered
        
        print(f"\033[92mCamera tracking enabled: body='{body_name}', distance={distance}, "
              f"filter_xy={filter_xy}, filter_z={filter_z}\033[0m")
        return True
    
    def disable_camera_tracking(self):
        """Disable camera tracking"""
        self._camera_track_body = None
        self._camera_track_body_id = -1
        self._camera_lookat_filtered = None
        print(f"\033[93mCamera tracking disabled\033[0m")
    
    def set_camera_track_filter(self, filter_xy: float = None, filter_z: float = None):
        """
        Set camera tracking filter parameters
        
        Args:
            filter_xy: Low-pass filter alpha for XY axes (0~1, lower = smoother)
            filter_z: Low-pass filter alpha for Z axis (stronger = more jitter reduction)
        """
        if filter_xy is not None:
            self._camera_track_filter_xy = np.clip(filter_xy, 0.001, 1.0)
        if filter_z is not None:
            self._camera_track_filter_z = np.clip(filter_z, 0.001, 1.0)
        print(f"\033[92mCamera filter updated: xy={self._camera_track_filter_xy}, z={self._camera_track_filter_z}\033[0m")
    
    def set_camera_track_distance(self, distance: float):
        """
        Set camera tracking distance
        
        Args:
            distance: Distance from the tracked body
        """
        self._camera_track_distance = distance
        if self.viewer is not None:
            self.viewer.cam.distance = distance
    
    def _update_camera_tracking(self):
        """Update camera lookat position to track the target body with filtering"""
        if self._camera_track_body_id < 0 or self.viewer is None:
            return
        
        # Get target body position
        body_pos = self.data.xpos[self._camera_track_body_id]
        
        # Initialize filter state if needed
        if self._camera_lookat_filtered is None:
            self._camera_lookat_filtered = body_pos.copy()
        
        # Apply low-pass filter: new = alpha * target + (1 - alpha) * old
        # XY axes: moderate filtering
        alpha_xy = self._camera_track_filter_xy
        self._camera_lookat_filtered[0] = alpha_xy * body_pos[0] + (1 - alpha_xy) * self._camera_lookat_filtered[0]
        self._camera_lookat_filtered[1] = alpha_xy * body_pos[1] + (1 - alpha_xy) * self._camera_lookat_filtered[1]
        
        # Z axis: stronger filtering to reduce walking jitter
        alpha_z = self._camera_track_filter_z
        self._camera_lookat_filtered[2] = alpha_z * body_pos[2] + (1 - alpha_z) * self._camera_lookat_filtered[2]
        
        # Update camera lookat (attitude unchanged)
        self.viewer.cam.lookat[:] = self._camera_lookat_filtered
    
    def is_running(self) -> bool:
        """Check if viewer is still running"""
        if self.viewer is None:
            return True
        return self.viewer.is_running()
    
    def close(self):
        """Close viewer"""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
