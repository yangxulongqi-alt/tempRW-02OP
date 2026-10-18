"""
Viser-based Model Viewer ROS2 Node

Web-based visualization of robot state using Viser.
Subscribes to JointState and IMU messages.

Subscribes:
  - /joint_states (sensor_msgs/JointState)
  - /imu/data (sensor_msgs/Imu)

Usage:
  ros2 run mjsim viser_model_viewer --ros-args -p model:=panda
  ros2 run mjsim viser_model_viewer --ros-args -p model:=/path/to/model.xml
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field

import mujoco
import numpy as np
import rclpy
import trimesh
import trimesh.visual
import trimesh.visual.material
import viser
import viser.transforms as vtf
from mujoco import mj_id2name, mjtGeom, mjtObj
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, JointState

from mjsim.cfgs import get_model_config, get_model_path


# =============================================================================
# Mesh Conversion Utilities
# =============================================================================


def mujoco_mesh_to_trimesh(
    mj_model: mujoco.MjModel, geom_idx: int
) -> trimesh.Trimesh:
    """Convert a MuJoCo mesh geometry to a trimesh."""
    mesh_id = mj_model.geom_dataid[geom_idx]

    vert_start = int(mj_model.mesh_vertadr[mesh_id])
    vert_count = int(mj_model.mesh_vertnum[mesh_id])
    face_start = int(mj_model.mesh_faceadr[mesh_id])
    face_count = int(mj_model.mesh_facenum[mesh_id])

    vertices = mj_model.mesh_vert[vert_start : vert_start + vert_count].copy()
    faces = mj_model.mesh_face[face_start : face_start + face_count].copy()

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)

    # Apply PBR material for better lighting
    matid = mj_model.geom_matid[geom_idx]
    if matid >= 0 and matid < mj_model.nmat:
        rgba = mj_model.mat_rgba[matid]
    else:
        # Use geom_rgba if no material
        rgba = mj_model.geom_rgba[geom_idx]
    
    material = trimesh.visual.material.PBRMaterial(
        baseColorFactor=rgba,
        metallicFactor=0.1,
        roughnessFactor=0.5,  # Lower roughness for better light reflection
        normalTexture=None,
        emissiveFactor=(0.15, 0.15, 0.15),  # Self-illumination to prevent dark areas
    )
    
    mesh.visual = trimesh.visual.TextureVisuals(material=material)

    return mesh


def create_primitive_mesh(mj_model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh:
    """Create a mesh for primitive geom types (sphere, box, capsule, cylinder, plane)."""
    size = mj_model.geom_size[geom_id]
    geom_type = mj_model.geom_type[geom_id]
    rgba = mj_model.geom_rgba[geom_id].copy()

    material = trimesh.visual.material.PBRMaterial(
        baseColorFactor=rgba,
        metallicFactor=0.1,
        roughnessFactor=0.5,  # Lower roughness for better light reflection
        normalTexture=None,
        emissiveFactor=(0.15, 0.15, 0.15),  # Self-illumination to prevent dark areas
    )

    if geom_type == mjtGeom.mjGEOM_SPHERE:
        mesh = trimesh.creation.icosphere(radius=size[0], subdivisions=2)
    elif geom_type == mjtGeom.mjGEOM_BOX:
        mesh = trimesh.creation.box(extents=2.0 * size)
    elif geom_type == mjtGeom.mjGEOM_CAPSULE:
        mesh = trimesh.creation.capsule(radius=size[0], height=2.0 * size[1])
    elif geom_type == mjtGeom.mjGEOM_CYLINDER:
        mesh = trimesh.creation.cylinder(radius=size[0], height=2.0 * size[1])
    elif geom_type == mjtGeom.mjGEOM_PLANE:
        mesh = trimesh.creation.box((20, 20, 0.01))
    elif geom_type == mjtGeom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=1.0)
        mesh.apply_scale(size)
    else:
        # Fallback to small sphere for unsupported types
        mesh = trimesh.creation.icosphere(radius=0.01, subdivisions=1)

    mesh.visual = trimesh.visual.TextureVisuals(material=material)
    return mesh


def merge_geoms(mj_model: mujoco.MjModel, geom_ids: list[int]) -> trimesh.Trimesh:
    """Merge multiple geoms into a single trimesh."""
    meshes = []
    for geom_id in geom_ids:
        geom_type = mj_model.geom_type[geom_id]

        if geom_type == mjtGeom.mjGEOM_MESH:
            mesh = mujoco_mesh_to_trimesh(mj_model, geom_id)
        else:
            mesh = create_primitive_mesh(mj_model, geom_id)

        # Apply local transform
        pos = mj_model.geom_pos[geom_id]
        quat = mj_model.geom_quat[geom_id]
        transform = np.eye(4)
        transform[:3, :3] = vtf.SO3(quat).as_matrix()
        transform[:3, 3] = pos
        mesh.apply_transform(transform)
        meshes.append(mesh)

    if len(meshes) == 1:
        return meshes[0]
    return trimesh.util.concatenate(meshes)


def is_fixed_body(mj_model: mujoco.MjModel, body_id: int) -> bool:
    """Check if a body is fixed (welded to world)."""
    is_weld = mj_model.body_weldid[body_id] == 0
    root_id = mj_model.body_rootid[body_id]
    root_is_mocap = mj_model.body_mocapid[root_id] >= 0
    return is_weld and not root_is_mocap


def get_body_name(mj_model: mujoco.MjModel, body_id: int) -> str:
    """Get body name with fallback to ID-based name."""
    body_name = mj_id2name(mj_model, mjtObj.mjOBJ_BODY, body_id)
    if not body_name:
        body_name = f"body_{body_id}"
    return body_name


# =============================================================================
# MARK: Viser场景
# =============================================================================


@dataclass
class ViserMujocoScene:
    """Manages Viser visualization for a MuJoCo model."""

    server: viser.ViserServer
    mj_model: mujoco.MjModel
    mj_data: mujoco.MjData

    # Handles
    mesh_handles: dict[int, viser.MeshHandle | viser.BatchedMeshHandle] = field(
        default_factory=dict
    )
    fixed_bodies_frame: viser.SceneNodeHandle = field(init=False)

    # Camera tracking
    camera_tracking_enabled: bool = False
    _tracked_body_id: int | None = field(init=False, default=None)

    @staticmethod
    def create(
        server: viser.ViserServer,
        mj_model: mujoco.MjModel,
    ) -> "ViserMujocoScene":
        """Create and populate scene with geometry."""
        mj_data = mujoco.MjData(mj_model)

        scene = ViserMujocoScene(
            server=server,
            mj_model=mj_model,
            mj_data=mj_data,
        )

        # Configure environment lighting with better settings
        server.scene.configure_environment_map(
            environment_intensity=3.0,  # Bright environment lighting
        )
        
        # Configure environment lighting only (no directional lights in this Viser version)

        # Create frame for fixed world geometry
        scene.fixed_bodies_frame = server.scene.add_frame("/fixed_bodies", show_axes=False)

        # Add fixed geometry (planes, terrain, etc.)
        scene._add_fixed_geometry()

        # Create mesh handles for movable bodies
        scene._create_mesh_handles()

        # Find first non-fixed body for camera tracking
        for body_id in range(mj_model.nbody):
            if not is_fixed_body(mj_model, body_id):
                scene._tracked_body_id = body_id
                break

        return scene

    def _add_fixed_geometry(self) -> None:
        """Add fixed world geometry to the scene."""
        body_geoms: dict[int, list[int]] = {}

        for i in range(self.mj_model.ngeom):
            body_id = self.mj_model.geom_bodyid[i]
            body_geoms.setdefault(body_id, []).append(i)

        for body_id, geom_ids in body_geoms.items():
            if not is_fixed_body(self.mj_model, body_id):
                continue

            body_name = get_body_name(self.mj_model, body_id)

            nonplane_geom_ids: list[int] = []
            for geom_id in geom_ids:
                geom_type = self.mj_model.geom_type[geom_id]
                if geom_type == mjtGeom.mjGEOM_PLANE:
                    geom_name = mj_id2name(self.mj_model, mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
                    self.server.scene.add_grid(
                        f"/fixed_bodies/{body_name}/{geom_name}",
                        width=50.0,
                        height=50.0,
                        infinite_grid=True,
                        fade_distance=20.0,
                        shadow_opacity=0.2,
                        position=self.mj_model.geom_pos[geom_id],
                        wxyz=self.mj_model.geom_quat[geom_id],
                    )
                else:
                    nonplane_geom_ids.append(geom_id)

            if nonplane_geom_ids:
                self.server.scene.add_mesh_trimesh(
                    f"/fixed_bodies/{body_name}",
                    merge_geoms(self.mj_model, nonplane_geom_ids),
                    cast_shadow=False,
                    receive_shadow=0.1,
                    position=self.mj_model.body(body_id).pos,
                    wxyz=self.mj_model.body(body_id).quat,
                    visible=True,
                )

    def _create_mesh_handles(self) -> None:
        """Create mesh handles for each non-fixed body."""
        body_geoms: dict[int, list[int]] = {}

        for i in range(self.mj_model.ngeom):
            body_id = self.mj_model.geom_bodyid[i]
            if is_fixed_body(self.mj_model, body_id):
                continue
            # Skip planes for movable bodies
            if self.mj_model.geom_type[i] == mjtGeom.mjGEOM_PLANE:
                continue
            body_geoms.setdefault(body_id, []).append(i)

        for body_id, geom_ids in body_geoms.items():
            body_name = get_body_name(self.mj_model, body_id)
            mesh = merge_geoms(self.mj_model, geom_ids)

            handle = self.server.scene.add_mesh_trimesh(
                f"/bodies/{body_name}",
                mesh,
                cast_shadow=False,
                receive_shadow=0.1,
                visible=True,
            )
            self.mesh_handles[body_id] = handle

    def update(self, mj_data: mujoco.MjData) -> None:
        """Update scene from MuJoCo data."""
        scene_offset = np.zeros(3)
        if self.camera_tracking_enabled and self._tracked_body_id is not None:
            tracked_pos = mj_data.xpos[self._tracked_body_id].copy()
            scene_offset = -tracked_pos

        self.fixed_bodies_frame.position = scene_offset

        for body_id, handle in self.mesh_handles.items():
            pos = mj_data.xpos[body_id] + scene_offset
            mat = mj_data.xmat[body_id].reshape(3, 3)
            quat = vtf.SO3.from_matrix(mat).wxyz

            handle.position = pos
            handle.wxyz = quat

    def create_gui(self) -> None:
        """Create GUI controls."""
        with self.server.gui.add_folder("Visualization"):
            cb_tracking = self.server.gui.add_checkbox(
                "Camera Tracking",
                initial_value=self.camera_tracking_enabled,
                hint="Track the robot with the camera",
            )

            @cb_tracking.on_update
            def _(_) -> None:
                self.camera_tracking_enabled = cb_tracking.value


# =============================================================================
# MARK: ROS2 Node
# =============================================================================


class ViserModelViewerNode(Node):
    """Viser-based Model Viewer Node."""

    def __init__(self):
        super().__init__("viser_model_viewer_node")

        # Parameters
        self.declare_parameter("model", "panda")
        self.declare_parameter("update_rate", 60)
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("port", 8080)

        model_name = self.get_parameter("model").get_parameter_value().string_value
        update_rate = self.get_parameter("update_rate").get_parameter_value().integer_value
        joint_topic = self.get_parameter("joint_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        # Resolve model path and get config
        try:
            xml_path = get_model_path(model_name)
        except ValueError as e:
            self.get_logger().error(f"\033[91m{e}\033[0m")
            raise

        # Get model config for motor_dir
        self._model_config = get_model_config(model_name)
        # self._motor_dir_map = {}
        # if (
        #     self._model_config is not None
        #     and self._model_config.motor_names
        #     and self._model_config.motor_dir
        # ):
        #     for name, direction in zip(
        #         self._model_config.motor_names, self._model_config.motor_dir
        #     ):
        #         self._motor_dir_map[name] = direction
        #     self.get_logger().info(
        #         f"\033[93mMotor direction mapping loaded: {len(self._motor_dir_map)} joints\033[0m"
        #     )

        # Load MuJoCo model
        if not os.path.exists(xml_path):
            self.get_logger().error(f"\033[91mModel file not found: {xml_path}\033[0m")
            sys.exit(1)

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Build actuator name -> joint qpos/dof address mapping
        self.actuator_num = self.model.nu
        self._actuator_name_to_idx = {}
        self._actuator_jnt_qposadr = np.zeros(self.actuator_num, dtype=np.int32)
        self._actuator_jnt_dofadr = np.zeros(self.actuator_num, dtype=np.int32)

        for i in range(self.actuator_num):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator_{i}"
            self._actuator_name_to_idx[name] = i
            trnid = self.model.actuator_trnid[i, 0]
            self._actuator_jnt_qposadr[i] = self.model.jnt_qposadr[trnid]
            self._actuator_jnt_dofadr[i] = self.model.jnt_dofadr[trnid]

        # Find floating base (if exists)
        self._has_floating_base = False
        self._base_qposadr = 0
        if self.model.nq > self.actuator_num:
            if self.model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
                self._has_floating_base = True
                self._base_qposadr = self.model.jnt_qposadr[0]
                self.get_logger().info("\033[93mFloating base detected\033[0m")

        # Create Viser server (bind to 0.0.0.0 for external access)
        self._server = viser.ViserServer(host="0.0.0.0", port=port)
        import socket
        # Get local IP address
        try:
            # Create a socket to determine local IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
        except Exception:
            local_ip = "127.0.0.1"
        
        self.get_logger().info(f"\033[92mViser server started at http://0.0.0.0:{port}\033[0m")
        self.get_logger().info(f"\033[92mExternal access: http://{local_ip}:{port}\033[0m")

        # Create scene
        self._scene = ViserMujocoScene.create(self._server, self.model)
        self._scene.create_gui()

        # Add status panel
        self._status_html = self._server.gui.add_markdown("### Status\nWaiting for data...")

        # State buffers
        self._joint_state = None
        self._imu_data = None
        self._last_joint_time = 0.0
        self._last_imu_time = 0.0

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, joint_topic, self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        # Update timer
        update_period = 1.0 / update_rate
        self.update_timer = self.create_timer(update_period, self.update_viewer)

        # Status timer
        self._start_time = time.time()
        self._last_status_time = 0.0
        self._step_count = 0

        self._print_init_info(xml_path, update_rate, joint_topic, imu_topic, port)

    def _print_init_info(
        self, xml_path: str, update_rate: int, joint_topic: str, imu_topic: str, port: int
    ):
        """Print initialization info."""
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')
        self.get_logger().info("\033[96mViserModelViewerNode Started\033[0m")
        self.get_logger().info(f"\033[93mModel:\033[0m {xml_path}")
        self.get_logger().info(f"\033[93mActuators:\033[0m {self.actuator_num}")
        self.get_logger().info(f"\033[93mFloating Base:\033[0m {self._has_floating_base}")
        self.get_logger().info(f"\033[93mUpdate Rate:\033[0m {update_rate} Hz")
        self.get_logger().info(f"\033[93mWeb UI:\033[0m http://localhost:{port}")
        self.get_logger().info("\033[93mSubscribing:\033[0m")
        self.get_logger().info(f"  - {joint_topic} (JointState)")
        self.get_logger().info(f"  - {imu_topic} (Imu)")
        self.get_logger().info(f'\033[96m{"="*50}\033[0m')

    def joint_callback(self, msg: JointState):
        """Handle incoming joint state."""
        self._joint_state = msg
        self._last_joint_time = time.time()

    def imu_callback(self, msg: Imu):
        """Handle incoming IMU data."""
        self._imu_data = msg
        self._last_imu_time = time.time()

    def update_viewer(self):
        """Update MuJoCo model state and sync viewer."""
        # Apply joint state
        if self._joint_state is not None:
            self._apply_joint_state(self._joint_state)

        # Apply IMU orientation (for floating base)
        if self._imu_data is not None and self._has_floating_base:
            self._apply_imu_orientation(self._imu_data)

        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)

        # Update Viser scene
        self._scene.update(self.data)

        # Update status
        self._step_count += 1
        self._update_status()

    def _apply_joint_state(self, msg: JointState):
        """Apply joint positions from JointState message."""
        for i, name in enumerate(msg.name):
            if name in self._actuator_name_to_idx:
                idx = self._actuator_name_to_idx[name]
                qpos_adr = self._actuator_jnt_qposadr[idx]
                dof_adr = self._actuator_jnt_dofadr[idx]

                # motor_dir = self._motor_dir_map.get(name, 1)

                if i < len(msg.position):
                    self.data.qpos[qpos_adr] = msg.position[i] # * motor_dir

                if i < len(msg.velocity):
                    self.data.qvel[dof_adr] = msg.velocity[i] # * motor_dir

    def _apply_imu_orientation(self, msg: Imu):
        """Apply IMU orientation to floating base."""
        if not self._has_floating_base:
            return

        quat_ros = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])

        # Convert to MuJoCo format: w, x, y, z
        quat_mujoco = np.array([quat_ros[3], quat_ros[0], quat_ros[1], quat_ros[2]])

        qpos_adr = self._base_qposadr
        self.data.qpos[qpos_adr + 3 : qpos_adr + 7] = quat_mujoco

    def _update_status(self):
        """Update status display."""
        now = time.time()
        if now - self._last_status_time < 1.0:
            return
        self._last_status_time = now

        elapsed = now - self._start_time
        joint_age = now - self._last_joint_time if self._last_joint_time > 0 else float("inf")
        imu_age = now - self._last_imu_time if self._last_imu_time > 0 else float("inf")

        joint_status = f"{joint_age:.1f}s ago" if joint_age < 100 else "No data"
        imu_status = f"{imu_age:.1f}s ago" if imu_age < 100 else "No data"

        self._status_html.content = f"""
### Status
- **Elapsed:** {elapsed:.1f}s
- **Steps:** {self._step_count}
- **Joint:** {joint_status}
- **IMU:** {imu_status}
"""

    def destroy_node(self):
        """Clean up resources."""
        self._server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ViserModelViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[91mError: {e}\033[0m")
        import traceback

        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
