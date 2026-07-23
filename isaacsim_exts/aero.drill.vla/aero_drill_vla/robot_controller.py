from __future__ import annotations

import math
from typing import Any

import isaacsim.core.experimental.utils.transform as transform_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import Cylinder, Mesh
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)

from .scene_builder import ROBOT_ROOT, TOOL_ROOT, TOOL_TCP_OFFSET_M, set_tool_pose


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_LABELS = (
    "J1 Base",
    "J2 Shoulder",
    "J3 Elbow",
    "J4 Wrist 1",
    "J5 Wrist 2",
    "J6 Wrist 3",
)
DEFAULT_JOINT_POSITIONS = np.deg2rad(
    np.array([-127.914, -54.595, -92.824, -32.580, 37.914, 90.0], dtype=np.float32)
)


def _rotation_from_tool_axis(inward_axis: np.ndarray) -> np.ndarray:
    """Build a stable tool rotation whose local +Z is the drilling direction."""
    z_axis = np.asarray(inward_axis, dtype=np.float64)
    z_axis /= max(np.linalg.norm(z_axis), 1e-9)
    reference_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    y_axis = reference_up - np.dot(reference_up, z_axis) * z_axis
    if np.linalg.norm(y_axis) < 1e-6:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    return np.column_stack((x_axis, y_axis, z_axis)).astype(np.float32)


def _matrix_to_rpy_degrees(rotation: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(float(rotation[0, 0] ** 2 + rotation[1, 0] ** 2))
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        pitch = math.atan2(float(-rotation[2, 0]), sy)
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(float(-rotation[1, 2]), float(rotation[1, 1]))
        pitch = math.atan2(float(-rotation[2, 0]), sy)
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class UR10eMotionController:
    """Official UR10e articulation controlled by collision-aware cuMotion RMPflow."""

    def __init__(self, stage) -> None:
        self.stage = stage
        self.articulation: Articulation | None = None
        self.cumotion_robot: Any | None = None
        self.controller: RmpFlowController | None = None
        self.world_binding: mg.WorldBinding | None = None
        self.world_interface: CumotionWorldInterface | None = None
        self.tool_frame = "tool0"
        self.time = 0.0
        self.ready = False
        self.controller_reset = False
        self.collision_enabled = False
        self.collision_world_count = 0
        self.last_error = ""
        self.commanded_tcp = np.array([0.60, 0.0, 0.80], dtype=np.float32)
        self.commanded_outward = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        self.current_tcp = np.array(self.commanded_tcp, copy=True)
        self.current_tool0 = np.array([0.25, 0.0, 0.80], dtype=np.float32)
        self.current_rotation = _rotation_from_tool_axis(-self.commanded_outward)
        self.tcp_error_mm = 0.0
        self.joint_positions = np.zeros(6, dtype=np.float32)
        self.joint_velocities = np.zeros(6, dtype=np.float32)
        self.joint_lower_limits = np.full(6, -2.0 * math.pi, dtype=np.float32)
        self.joint_upper_limits = np.full(6, 2.0 * math.pi, dtype=np.float32)

    @property
    def mode(self) -> str:
        if not self.ready:
            return "UR10e initializing"
        collision = "collision world ON" if self.collision_enabled else "collision fallback"
        return f"UR10e + cuMotion RMPflow | {collision}"

    @property
    def tcp_rpy_deg(self) -> tuple[float, float, float]:
        return _matrix_to_rpy_degrees(self.current_rotation)

    def ensure_initialized(self) -> bool:
        if self.ready:
            return True
        if not self.stage.GetPrimAtPath(f"{ROBOT_ROOT}/wrist_3_link").IsValid():
            self.last_error = "Waiting for the official UR10e asset"
            return False
        if not SimulationManager.is_simulating():
            self.last_error = "Start Simulation Play to initialize UR10e physics"
            return False
        try:
            self.articulation = Articulation(ROBOT_ROOT)
            if tuple(self.articulation.dof_names) != JOINT_NAMES:
                raise RuntimeError(
                    f"UR10e DOF mismatch: expected {JOINT_NAMES}, got {self.articulation.dof_names}"
                )
            self.articulation.set_default_state(dof_positions=DEFAULT_JOINT_POSITIONS)
            self.articulation.set_dof_positions(DEFAULT_JOINT_POSITIONS)
            self.articulation.set_dof_position_targets(DEFAULT_JOINT_POSITIONS)

            lower, upper = self.articulation.get_dof_limits()
            self.joint_lower_limits = lower.numpy().reshape(-1)[:6]
            self.joint_upper_limits = upper.numpy().reshape(-1)[:6]

            self.cumotion_robot = load_cumotion_supported_robot("ur10")
            self.tool_frame = self.cumotion_robot.robot_description.tool_frame_names()[0]
            self.world_interface = CumotionWorldInterface()
            self._initialize_collision_world()
            self.controller = RmpFlowController(
                cumotion_robot=self.cumotion_robot,
                cumotion_world_interface=self.world_interface,
                robot_joint_space=self.articulation.dof_names,
                robot_site_space=[self.tool_frame],
                tool_frame=self.tool_frame,
            )
            config = self.controller.get_rmp_flow_config()
            config.set_param("cspace_target_rmp/metric_scalar", 1.0)
            self.ready = True
            self._refresh_state()
            self.last_error = ""
            return True
        except Exception as error:
            self.last_error = f"UR10e initialization failed: {error}"
            self.articulation = None
            self.controller = None
            self.cumotion_robot = None
            return False

    def _initialize_collision_world(self) -> None:
        assert self.articulation is not None
        assert self.world_interface is not None
        robot_position, robot_orientation = self.articulation.get_world_poses()
        obstacles = mg.SceneQuery().get_prims_in_aabb(
            search_box_origin=robot_position.numpy()[0],
            search_box_minimum=[-2.0, -2.0, -1.0],
            search_box_maximum=[2.0, 2.0, 3.0],
            tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[ROBOT_ROOT, TOOL_ROOT],
        )
        strategy = mg.ObstacleStrategy()
        for prim_type in (Mesh, Cylinder):
            strategy.set_default_configuration(
                prim_type,
                mg.ObstacleConfiguration("obb", 0.008),
            )
        self.world_binding = mg.WorldBinding(
            world_interface=self.world_interface,
            obstacle_strategy=strategy,
            tracked_prims=obstacles,
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )
        self.world_binding.initialize()
        self.world_interface.update_world_to_robot_root_transforms(
            poses=(robot_position, robot_orientation)
        )
        self.world_binding.synchronize_transforms()
        self.collision_world_count = len(obstacles)
        self.collision_enabled = self.collision_world_count > 0

    def _estimated_state(self) -> mg.RobotState:
        assert self.articulation is not None
        names = self.articulation.dof_names
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, self.articulation.get_dof_positions()),
                velocities=(names, self.articulation.get_dof_velocities()),
            )
        )

    def _target_state(self, tool0_position: np.ndarray, rotation: np.ndarray) -> mg.RobotState:
        orientation = transform_utils.rotation_matrix_to_quaternion(rotation).numpy().reshape(-1)
        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=[self.tool_frame],
                positions=(
                    [self.tool_frame],
                    wp.array([tool0_position.tolist()], dtype=wp.float32),
                ),
                orientations=(
                    [self.tool_frame],
                    wp.array([orientation.tolist()], dtype=wp.float32),
                ),
            )
        )

    def command_tcp(self, tcp_position, outward_axis, dt: float) -> bool:
        if not self.ensure_initialized():
            return False
        assert self.articulation is not None
        assert self.controller is not None
        assert self.world_binding is not None

        self.commanded_tcp = np.asarray(tcp_position, dtype=np.float32)
        self.commanded_outward = np.asarray(outward_axis, dtype=np.float32)
        self.commanded_outward /= max(np.linalg.norm(self.commanded_outward), 1e-9)
        inward_axis = -self.commanded_outward
        rotation = _rotation_from_tool_axis(inward_axis)
        tool0_position = self.commanded_tcp - inward_axis * TOOL_TCP_OFFSET_M
        target = self._target_state(tool0_position, rotation)

        if not self.controller_reset:
            self.controller_reset = self.controller.reset(
                self._estimated_state(),
                target,
                t=0.0,
            )
            self.time = 0.0
            if not self.controller_reset:
                self.last_error = "cuMotion RMPflow reset failed"
                return False

        robot_pose = self.articulation.get_world_poses()
        self.world_interface.update_world_to_robot_root_transforms(poses=robot_pose)
        self.world_binding.synchronize_transforms()

        self.time += max(float(dt), 1.0 / 240.0)
        desired = self.controller.forward(self._estimated_state(), target, self.time)
        if desired is not None and desired.joints.positions is not None:
            self.articulation.set_dof_position_targets(
                positions=desired.joints.positions,
                dof_indices=desired.joints.position_indices,
            )
        self._refresh_state()
        self.tcp_error_mm = float(np.linalg.norm(self.current_tcp - self.commanded_tcp) * 1000.0)
        return True

    def _refresh_state(self) -> None:
        if self.articulation is None or self.cumotion_robot is None:
            return
        self.joint_positions = self.articulation.get_dof_positions().numpy().reshape(-1)[:6]
        self.joint_velocities = self.articulation.get_dof_velocities().numpy().reshape(-1)[:6]
        pose = self.cumotion_robot.kinematics.pose(self.joint_positions, self.tool_frame)
        matrix = np.asarray(pose.matrix(), dtype=np.float64)
        self.current_tool0 = np.asarray(pose.translation, dtype=np.float32)
        self.current_rotation = matrix[:3, :3].astype(np.float32)
        self.current_tcp = (
            self.current_tool0
            + self.current_rotation @ np.array([0.0, 0.0, TOOL_TCP_OFFSET_M], dtype=np.float32)
        )
        orientation = (
            transform_utils.rotation_matrix_to_quaternion(self.current_rotation)
            .numpy()
            .reshape(-1)
        )
        set_tool_pose(
            self.stage,
            self.current_tool0.tolist(),
            orientation.tolist(),
        )

    def reset_home(self) -> None:
        if self.articulation is not None:
            self.articulation.set_dof_positions(DEFAULT_JOINT_POSITIONS)
            self.articulation.set_dof_position_targets(DEFAULT_JOINT_POSITIONS)
            self.controller_reset = False
            self.time = 0.0
            self._refresh_state()

    def joint_rows(self) -> list[dict[str, float | str]]:
        rows: list[dict[str, float | str]] = []
        for index, (name, label) in enumerate(zip(JOINT_NAMES, JOINT_LABELS)):
            rows.append(
                {
                    "name": name,
                    "label": label,
                    "position_deg": math.degrees(float(self.joint_positions[index])),
                    "velocity_deg_s": math.degrees(float(self.joint_velocities[index])),
                    "lower_deg": math.degrees(float(self.joint_lower_limits[index])),
                    "upper_deg": math.degrees(float(self.joint_upper_limits[index])),
                }
            )
        return rows
