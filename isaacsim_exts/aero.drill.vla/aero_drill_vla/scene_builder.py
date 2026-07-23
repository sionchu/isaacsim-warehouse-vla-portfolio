from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from .hole_policy import HOLE_IDS


SCENE_ROOT = "/World/AeroDrillVLA"
ROBOT_ROOT = f"{SCENE_ROOT}/UR10e"
ROBOT_ASSET_RELATIVE_PATH = "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
DRPE_ROOT = f"{SCENE_ROOT}/AircraftPanel/DRPE"
HOLE_ROOT = f"{DRPE_ROOT}/Holes"
STATUS_ROOT = f"{SCENE_ROOT}/StatusLights"
FRAMES_ROOT = f"{SCENE_ROOT}/CoordinateFrames"
ACTIVE_HOLE_FRAME = f"{FRAMES_ROOT}/ActiveHole"
BASE_FRAME = f"{FRAMES_ROOT}/UR_Base"
TOOL_ROOT = f"{SCENE_ROOT}/AeroDrillTool"
TCP_FRAME = f"{TOOL_ROOT}/TCP"

TOOL_TCP_OFFSET_M = 0.35
PANEL_X_M = 0.95
PANEL_RADIUS_M = 2.40

JOINT_LINKS = (
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
)
JOINT_FRAME_PATHS = tuple(f"{ROBOT_ROOT}/{link}/JointFrame" for link in JOINT_LINKS)
FRAME_PATHS = (BASE_FRAME, ACTIVE_HOLE_FRAME, TCP_FRAME, *JOINT_FRAME_PATHS)


@dataclass(frozen=True)
class HoleFrame:
    hole_id: str
    center: tuple[float, float, float]
    outward: tuple[float, float, float]
    position_error_mm: float
    normal_error_deg: float
    material_stack: str


POSITION_ERRORS = (0.18, 0.42, 0.95, 0.27, 0.61, 1.15, 0.33, 0.74, 0.22, 0.88)
NORMAL_ERRORS = (0.12, 0.31, 0.92, 0.24, 0.58, 1.08, 0.29, 0.67, 0.18, 0.81)
MATERIAL_STACKS = (
    "AL-CFRP",
    "CFRP-TI",
    "AL-LI",
    "AL-CFRP",
    "CFRP",
    "CFRP-TI",
    "AL-LI",
    "AL-CFRP",
    "CFRP",
    "CFRP-TI",
)


def _surface_x(y: float) -> float:
    return PANEL_X_M + (y * y) / (2.0 * PANEL_RADIUS_M)


def hole_frames() -> dict[str, HoleFrame]:
    """Return ten dimensionally plausible drill-bushing target frames in metres."""
    frames: dict[str, HoleFrame] = {}
    y_values = (-0.36, -0.18, 0.0, 0.18, 0.36)
    z_values = (0.62, 0.90)
    for index, hole_id in enumerate(HOLE_IDS):
        row, column = divmod(index, 5)
        y = y_values[column]
        z = z_values[row]
        derivative = y / PANEL_RADIUS_M
        length = math.sqrt(1.0 + derivative * derivative)
        outward = (-1.0 / length, derivative / length, 0.0)
        skin = Gf.Vec3d(_surface_x(y), y, z)
        normal = Gf.Vec3d(*outward)
        center = skin + normal * 0.016
        frames[hole_id] = HoleFrame(
            hole_id=hole_id,
            center=(center[0], center[1], center[2]),
            outward=outward,
            position_error_mm=POSITION_ERRORS[index],
            normal_error_deg=NORMAL_ERRORS[index],
            material_stack=MATERIAL_STACKS[index],
        )
    return frames


def _apply_collision(prim) -> None:
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)


def _cube(stage, path: str, position, scale, color, collision: bool = False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.XformCommonAPI(cube.GetPrim())
    transform.SetTranslate(Gf.Vec3d(*position))
    transform.SetScale(Gf.Vec3f(*scale))
    if collision:
        _apply_collision(cube.GetPrim())
    return cube.GetPrim()


def _sphere(stage, path: str, position, radius: float, color, collision: bool = False):
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)
    sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.XformCommonAPI(sphere.GetPrim()).SetTranslate(Gf.Vec3d(*position))
    if collision:
        _apply_collision(sphere.GetPrim())
    return sphere.GetPrim()


def _cylinder(
    stage,
    path: str,
    radius: float,
    color,
    *,
    height: float = 0.1,
    position=(0.0, 0.0, 0.0),
    collision: bool = False,
):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.XformCommonAPI(cylinder.GetPrim()).SetTranslate(Gf.Vec3d(*position))
    if collision:
        _apply_collision(cylinder.GetPrim())
    return cylinder.GetPrim()


def _alignment_quaternion(direction) -> Gf.Quatd:
    direction = Gf.Vec3d(*direction).GetNormalized()
    dot = max(-1.0, min(1.0, direction[2]))
    axis = Gf.Vec3d(-direction[1], direction[0], 0.0)
    axis_length = axis.GetLength()
    if axis_length < 1e-8:
        axis = Gf.Vec3d(1.0, 0.0, 0.0)
        angle_degrees = 0.0 if dot >= 0.0 else 180.0
    else:
        axis /= axis_length
        angle_degrees = math.degrees(math.acos(dot))
    half_angle = math.radians(angle_degrees) * 0.5
    return Gf.Quatd(math.cos(half_angle), axis * math.sin(half_angle))


def _set_segment(stage, path: str, start, end) -> None:
    start_v = Gf.Vec3d(*start)
    end_v = Gf.Vec3d(*end)
    vector = end_v - start_v
    length = max(vector.GetLength(), 1e-6)
    direction = vector / length
    midpoint = (start_v + end_v) * 0.5
    cylinder = UsdGeom.Cylinder(stage.GetPrimAtPath(path))
    cylinder.GetHeightAttr().Set(length)
    xformable = UsdGeom.Xformable(cylinder.GetPrim())
    transform_attr = cylinder.GetPrim().GetAttribute("xformOp:transform")
    if not transform_attr.IsValid():
        transform_attr = xformable.AddTransformOp().GetAttr()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(_alignment_quaternion(direction))
    matrix.SetTranslateOnly(midpoint)
    transform_attr.Set(matrix)


def _set_color(stage, path: str, color) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])


def _curved_mesh(
    stage,
    path: str,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    offset: float,
    color,
    *,
    collision: bool = False,
):
    mesh = UsdGeom.Mesh.Define(stage, path)
    columns = 24
    rows = 8
    points = []
    for row in range(rows + 1):
        z = z_min + (z_max - z_min) * row / rows
        for column in range(columns + 1):
            y = y_min + (y_max - y_min) * column / columns
            points.append(Gf.Vec3f(_surface_x(y) + offset, y, z))
    counts = []
    indices = []
    stride = columns + 1
    for row in range(rows):
        for column in range(columns):
            a = row * stride + column
            counts.append(4)
            indices.extend((a, a + 1, a + 1 + stride, a + stride))
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.XformCommonAPI(mesh.GetPrim())
    transform.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    transform.SetRotate(
        Gf.Vec3f(0.0, 0.0, 0.0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    transform.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
    if collision:
        _apply_collision(mesh.GetPrim())
    return mesh.GetPrim()


def _create_frame_gizmo(stage, path: str, length: float, radius: float) -> None:
    stage.DefinePrim(path, "Xform")
    _sphere(stage, f"{path}/Origin", (0.0, 0.0, 0.0), radius * 1.8, (0.92, 0.92, 0.92))
    for name, endpoint, color in (
        ("X", (length, 0.0, 0.0), (0.95, 0.10, 0.08)),
        ("Y", (0.0, length, 0.0), (0.10, 0.90, 0.18)),
        ("Z", (0.0, 0.0, length), (0.10, 0.34, 0.98)),
    ):
        _cylinder(stage, f"{path}/{name}", radius, color)
        _set_segment(stage, f"{path}/{name}", (0.0, 0.0, 0.0), endpoint)


def _build_aircraft_panel(stage) -> None:
    root = f"{SCENE_ROOT}/AircraftPanel"
    stage.DefinePrim(root, "Xform")
    _curved_mesh(
        stage,
        f"{root}/FuselageSkin",
        -0.85,
        0.85,
        0.28,
        1.30,
        0.0,
        (0.34, 0.45, 0.57),
        collision=True,
    )

    # Two narrow removable drill plates, each carrying five 10 mm pilot holes.
    _curved_mesh(
        stage,
        f"{DRPE_ROOT}/UpperPlate",
        -0.46,
        0.46,
        0.835,
        0.965,
        -0.012,
        (0.055, 0.070, 0.085),
        collision=True,
    )
    _curved_mesh(
        stage,
        f"{DRPE_ROOT}/LowerPlate",
        -0.46,
        0.46,
        0.555,
        0.685,
        -0.012,
        (0.055, 0.070, 0.085),
        collision=True,
    )

    # Representative aerospace substructure: frames, stringers, edge flanges and rivet rows.
    for y in (-0.82, -0.55, -0.28, 0.0, 0.28, 0.55, 0.82):
        _cube(
            stage,
            f"{root}/Frame_{int((y + 0.9) * 100):03d}",
            (1.065, y, 0.79),
            (0.030, 0.018, 0.54),
            (0.22, 0.28, 0.34),
            True,
        )
    for z in (0.34, 0.50, 0.76, 1.04, 1.24):
        _cube(
            stage,
            f"{root}/Stringer_{int(z * 100):03d}",
            (1.075, 0.0, z),
            (0.025, 0.90, 0.018),
            (0.40, 0.48, 0.55),
            True,
        )
    for y in (-0.74, 0.74):
        for z in (0.40, 0.56, 0.72, 0.88, 1.04, 1.20):
            _sphere(
                stage,
                f"{root}/Rivets/R_{int((y + 1) * 100)}_{int(z * 100)}",
                (_surface_x(y) - 0.008, y, z),
                0.005,
                (0.72, 0.76, 0.80),
            )

    _cube(stage, f"{root}/FootLeft", (1.11, -0.72, 0.02), (0.18, 0.12, 0.20), (0.09, 0.11, 0.14), True)
    _cube(stage, f"{root}/FootRight", (1.11, 0.72, 0.02), (0.18, 0.12, 0.20), (0.09, 0.11, 0.14), True)

    stage.DefinePrim(HOLE_ROOT, "Xform")
    stage.DefinePrim(STATUS_ROOT, "Xform")
    for hole_id, frame in hole_frames().items():
        center = Gf.Vec3d(*frame.center)
        outward = Gf.Vec3d(*frame.outward)
        stage.DefinePrim(f"{HOLE_ROOT}/{hole_id}", "Xform")

        bushing_path = f"{HOLE_ROOT}/{hole_id}/Bushing"
        _cylinder(stage, bushing_path, 0.014, (0.68, 0.72, 0.77))
        _set_segment(stage, bushing_path, center - outward * 0.010, center + outward * 0.010)

        bore_path = f"{HOLE_ROOT}/{hole_id}/Bore"
        _cylinder(stage, bore_path, 0.005, (0.006, 0.009, 0.012))
        _set_segment(stage, bore_path, center + outward * 0.011, center + outward * 0.014)

        axis_path = f"{HOLE_ROOT}/{hole_id}/Centerline"
        _cylinder(stage, axis_path, 0.0018, (0.10, 0.95, 0.48))
        _set_segment(stage, axis_path, center + outward * 0.018, center + outward * 0.18)

        status_position = center + outward * 0.020 + Gf.Vec3d(0.0, 0.0, 0.035)
        _sphere(stage, f"{STATUS_ROOT}/{hole_id}", status_position, 0.009, (0.27, 0.31, 0.36))


def _build_cell(stage) -> None:
    _cube(stage, f"{SCENE_ROOT}/Floor", (0.25, 0.0, -0.23), (1.70, 1.25, 0.05), (0.065, 0.075, 0.09), True)
    _cube(stage, f"{SCENE_ROOT}/RobotPedestal", (0.0, 0.0, -0.10), (0.34, 0.34, 0.10), (0.10, 0.12, 0.15), True)
    _cube(stage, f"{SCENE_ROOT}/RobotPedestalAccent", (0.0, 0.0, 0.005), (0.32, 0.32, 0.010), (0.05, 0.62, 0.70))
    _cube(stage, f"{SCENE_ROOT}/SafetyZone", (0.24, 0.0, -0.174), (1.18, 0.92, 0.006), (0.07, 0.23, 0.25))
    for y in (-0.93, 0.93):
        suffix = "N" if y < 0 else "P"
        _cube(stage, f"{SCENE_ROOT}/SafetyLineY_{suffix}", (0.24, y, -0.166), (1.20, 0.018, 0.008), (0.95, 0.70, 0.08))
    _cube(stage, f"{SCENE_ROOT}/ProcessCabinet", (-0.65, 0.92, 0.20), (0.25, 0.22, 0.38), (0.11, 0.13, 0.17), True)
    _cube(stage, f"{SCENE_ROOT}/ProcessCabinetScreen", (-0.65, 0.695, 0.31), (0.16, 0.008, 0.10), (0.04, 0.55, 0.66))
    _sphere(stage, f"{SCENE_ROOT}/ProcessCabinetBeacon", (-0.65, 0.92, 0.62), 0.032, (0.98, 0.55, 0.06))


def _build_ur10e(stage) -> None:
    robot = stage.DefinePrim(ROBOT_ROOT, "Xform")
    robot.GetReferences().AddReference(get_assets_root_path() + ROBOT_ASSET_RELATIVE_PATH)

    # R-eVo-inspired generic end effector. Its world pose is updated from cuMotion
    # forward kinematics so the rendered spindle axis exactly matches tool0.
    tool = stage.DefinePrim(TOOL_ROOT, "Xform")
    set_tool_pose(
        stage,
        (0.25, 0.0, 0.80),
        (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0),
    )

    _cylinder(
        stage,
        f"{TOOL_ROOT}/Motor",
        0.070,
        (0.72, 0.075, 0.055),
        height=0.14,
        position=(0.0, 0.0, 0.07),
    )
    _cylinder(
        stage,
        f"{TOOL_ROOT}/Gearbox",
        0.058,
        (0.18, 0.20, 0.23),
        height=0.08,
        position=(0.0, 0.0, 0.18),
    )
    _cylinder(
        stage,
        f"{TOOL_ROOT}/ClampBody",
        0.072,
        (0.94, 0.55, 0.06),
        height=0.09,
        position=(0.0, 0.0, 0.265),
    )
    _cylinder(
        stage,
        f"{TOOL_ROOT}/Nose",
        0.027,
        (0.70, 0.74, 0.78),
        height=0.07,
        position=(0.0, 0.0, 0.325),
    )
    _cylinder(
        stage,
        f"{TOOL_ROOT}/DrillBit",
        0.0048,
        (0.18, 0.20, 0.22),
        height=0.055,
        position=(0.0, 0.0, 0.368),
    )
    _cube(stage, f"{TOOL_ROOT}/VisionPod", (0.078, 0.0, 0.235), (0.038, 0.030, 0.032), (0.08, 0.11, 0.15))
    _sphere(stage, f"{TOOL_ROOT}/VisionLens", (0.080, 0.0, 0.270), 0.012, (0.10, 0.65, 0.95))

    _create_frame_gizmo(stage, TCP_FRAME, 0.085, 0.003)
    UsdGeom.XformCommonAPI(stage.GetPrimAtPath(TCP_FRAME)).SetTranslate(
        Gf.Vec3d(0.0, 0.0, TOOL_TCP_OFFSET_M)
    )
    _create_frame_gizmo(stage, BASE_FRAME, 0.16, 0.004)
    for path in JOINT_FRAME_PATHS:
        _create_frame_gizmo(stage, path, 0.065, 0.002)


def set_tool_pose(stage, tool0_position, tool0_orientation_wxyz) -> None:
    prim = stage.GetPrimAtPath(TOOL_ROOT)
    if not prim.IsValid():
        return
    orientation = Gf.Quatd(
        float(tool0_orientation_wxyz[0]),
        Gf.Vec3d(
            float(tool0_orientation_wxyz[1]),
            float(tool0_orientation_wxyz[2]),
            float(tool0_orientation_wxyz[3]),
        ),
    )
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(orientation)
    matrix.SetTranslateOnly(Gf.Vec3d(*tool0_position))
    transform_attr = prim.GetAttribute("xformOp:transform")
    if not transform_attr.IsValid():
        transform_attr = UsdGeom.Xformable(prim).AddTransformOp().GetAttr()
    transform_attr.Set(matrix)


def set_active_hole_frame(stage, hole_id: str | None) -> None:
    prim = stage.GetPrimAtPath(ACTIVE_HOLE_FRAME)
    if not prim.IsValid():
        return
    if not hole_id:
        UsdGeom.Imageable(prim).MakeInvisible()
        return
    frame = hole_frames()[hole_id]
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(_alignment_quaternion(frame.outward))
    matrix.SetTranslateOnly(Gf.Vec3d(*frame.center) + Gf.Vec3d(*frame.outward) * 0.025)
    transform_attr = prim.GetAttribute("xformOp:transform")
    if not transform_attr.IsValid():
        transform_attr = UsdGeom.Xformable(prim).AddTransformOp().GetAttr()
    transform_attr.Set(matrix)
    UsdGeom.Imageable(prim).MakeVisible()


def set_hole_status(stage, hole_id: str, status: str) -> None:
    colors = {
        "PENDING": (0.27, 0.31, 0.36),
        "ACTIVE": (1.0, 0.62, 0.08),
        "COMPLETE": (0.10, 0.88, 0.44),
        "FAILED": (0.96, 0.16, 0.18),
    }
    _set_color(stage, f"{STATUS_ROOT}/{hole_id}", colors.get(status, colors["PENDING"]))


def set_centerlines_visible(stage, visible: bool) -> None:
    for hole_id in HOLE_IDS:
        prim = stage.GetPrimAtPath(f"{HOLE_ROOT}/{hole_id}/Centerline")
        if prim.IsValid():
            imageable = UsdGeom.Imageable(prim)
            imageable.MakeVisible() if visible else imageable.MakeInvisible()


def set_frames_visible(stage, visible: bool) -> None:
    for path in FRAME_PATHS:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            imageable = UsdGeom.Imageable(prim)
            imageable.MakeVisible() if visible else imageable.MakeInvisible()


def build_scene(stage, output_path: str | Path | None = None) -> dict[str, str]:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    if stage.GetPrimAtPath(SCENE_ROOT).IsValid():
        stage.RemovePrim(SCENE_ROOT)
    stage.DefinePrim(SCENE_ROOT, "Xform")

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    dome = UsdLux.DomeLight.Define(stage, f"{SCENE_ROOT}/DomeLight")
    dome.CreateIntensityAttr(620.0)
    dome.CreateColorAttr(Gf.Vec3f(0.62, 0.72, 0.86))
    key = UsdLux.RectLight.Define(stage, f"{SCENE_ROOT}/KeyLight")
    key.CreateIntensityAttr(2600.0)
    key.CreateWidthAttr(2.8)
    key.CreateHeightAttr(2.0)
    UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(-0.6, -1.4, 2.7))

    _build_cell(stage)
    _build_aircraft_panel(stage)
    _build_ur10e(stage)
    _create_frame_gizmo(stage, ACTIVE_HOLE_FRAME, 0.10, 0.003)
    set_active_hole_frame(stage, None)

    if output_path:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output))

    return {
        "scene": SCENE_ROOT,
        "robot": ROBOT_ROOT,
        "tool": TOOL_ROOT,
        "tcp": TCP_FRAME,
        "drpe": DRPE_ROOT,
        "active_hole_frame": ACTIVE_HOLE_FRAME,
    }
