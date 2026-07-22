from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from .hole_policy import HOLE_IDS

SCENE_ROOT = "/World/AeroDrillVLA"
COBOT_ROOT = f"{SCENE_ROOT}/Cobot"
DRPE_ROOT = f"{SCENE_ROOT}/AircraftPanel/DRPE"
HOLE_ROOT = f"{DRPE_ROOT}/Holes"
STATUS_ROOT = f"{SCENE_ROOT}/StatusLights"
TOOL_ROOT = f"{COBOT_ROOT}/REvoInspiredTool"


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
MATERIAL_STACKS = ("AL-CFRP", "CFRP-TI", "AL-LI", "AL-CFRP", "CFRP", "CFRP-TI", "AL-LI", "AL-CFRP", "CFRP", "CFRP-TI")


def _surface_x(y: float) -> float:
    return 1.10 + 0.08 * (y / 1.20) ** 2


def hole_frames() -> dict[str, HoleFrame]:
    frames: dict[str, HoleFrame] = {}
    y_values = (-0.80, -0.40, 0.0, 0.40, 0.80)
    z_values = (0.88, 1.28)
    for index, hole_id in enumerate(HOLE_IDS):
        row, column = divmod(index, 5)
        y = y_values[column]
        z = z_values[row]
        derivative = (0.16 / (1.20**2)) * y
        length = math.sqrt(1.0 + derivative * derivative)
        outward = (-1.0 / length, derivative / length, 0.0)
        surface = Gf.Vec3d(_surface_x(y), y, z)
        normal = Gf.Vec3d(*outward)
        center = surface + normal * 0.035
        frames[hole_id] = HoleFrame(
            hole_id=hole_id,
            center=(center[0], center[1], center[2]),
            outward=outward,
            position_error_mm=POSITION_ERRORS[index],
            normal_error_deg=NORMAL_ERRORS[index],
            material_stack=MATERIAL_STACKS[index],
        )
    return frames


def _cube(stage, path: str, position, scale, color, collision: bool = False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.XformCommonAPI(cube.GetPrim())
    transform.SetTranslate(Gf.Vec3d(*position))
    transform.SetScale(Gf.Vec3f(*scale))
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def _sphere(stage, path: str, position, radius: float, color):
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)
    sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.XformCommonAPI(sphere.GetPrim()).SetTranslate(Gf.Vec3d(*position))
    return sphere.GetPrim()


def _cylinder(stage, path: str, radius: float, color):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(0.1)
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cylinder.GetPrim()


def _set_segment(stage, path: str, start, end) -> None:
    start_v = Gf.Vec3d(*start)
    end_v = Gf.Vec3d(*end)
    vector = end_v - start_v
    length = vector.GetLength()
    if length < 1e-6:
        vector = Gf.Vec3d(0.0, 0.0, 1.0)
        length = 1e-6
    direction = vector / length
    # Build an explicit axis-angle rotation. The two-vector Gf.Rotation overload
    # is ambiguous in some Kit Python bindings and can silently yield identity.
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
    rotation_quaternion = Gf.Quatd(math.cos(half_angle), axis * math.sin(half_angle))
    midpoint = (start_v + end_v) * 0.5
    prim = stage.GetPrimAtPath(path)
    cylinder = UsdGeom.Cylinder(prim)
    cylinder.GetHeightAttr().Set(length)
    xformable = UsdGeom.Xformable(prim)
    transform_attr = prim.GetAttribute("xformOp:transform")
    if not transform_attr.IsValid():
        transform_attr = xformable.AddTransformOp().GetAttr()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(rotation_quaternion)
    matrix.SetTranslateOnly(midpoint)
    transform_attr.Set(matrix)


def _set_position(stage, path: str, position) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*position))


def _set_color(stage, path: str, color) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])


def _curved_mesh(stage, path: str, y_min: float, y_max: float, z_min: float, z_max: float, offset: float, color):
    mesh = UsdGeom.Mesh.Define(stage, path)
    columns = 12
    rows = 4
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
    return mesh.GetPrim()


def _build_aircraft_panel(stage) -> None:
    root = f"{SCENE_ROOT}/AircraftPanel"
    stage.DefinePrim(root, "Xform")
    _curved_mesh(stage, f"{root}/Skin", -1.25, 1.25, 0.42, 1.72, 0.0, (0.24, 0.36, 0.48))
    _curved_mesh(stage, f"{DRPE_ROOT}/UpperPlate", -1.02, 1.02, 1.15, 1.41, -0.026, (0.055, 0.070, 0.085))
    _curved_mesh(stage, f"{DRPE_ROOT}/LowerPlate", -1.02, 1.02, 0.75, 1.01, -0.026, (0.055, 0.070, 0.085))

    # Structural frame and stringers behind the representative fuselage skin.
    for y in (-1.28, -0.86, -0.43, 0.0, 0.43, 0.86, 1.28):
        _cube(stage, f"{root}/Frame_{int((y + 1.3) * 100):03d}", (1.23, y, 1.07), (0.055, 0.025, 0.70), (0.35, 0.42, 0.49))
    for z in (0.48, 1.66):
        _cube(stage, f"{root}/Stringer_{int(z * 100):03d}", (1.22, 0.0, z), (0.05, 1.38, 0.045), (0.42, 0.49, 0.56))
    _cube(stage, f"{root}/FootLeft", (1.30, -1.12, 0.19), (0.28, 0.18, 0.19), (0.09, 0.11, 0.14), True)
    _cube(stage, f"{root}/FootRight", (1.30, 1.12, 0.19), (0.28, 0.18, 0.19), (0.09, 0.11, 0.14), True)

    stage.DefinePrim(HOLE_ROOT, "Xform")
    stage.DefinePrim(STATUS_ROOT, "Xform")
    for hole_id, frame in hole_frames().items():
        center = Gf.Vec3d(*frame.center)
        outward = Gf.Vec3d(*frame.outward)
        bushing_path = f"{HOLE_ROOT}/{hole_id}/Bushing"
        stage.DefinePrim(f"{HOLE_ROOT}/{hole_id}", "Xform")
        _cylinder(stage, bushing_path, 0.036, (0.66, 0.71, 0.76))
        _set_segment(stage, bushing_path, center - outward * 0.025, center + outward * 0.025)
        bore_path = f"{HOLE_ROOT}/{hole_id}/Bore"
        _cylinder(stage, bore_path, 0.020, (0.008, 0.012, 0.016))
        _set_segment(stage, bore_path, center + outward * 0.027, center + outward * 0.032)
        axis_path = f"{HOLE_ROOT}/{hole_id}/Centerline"
        _cylinder(stage, axis_path, 0.0035, (0.10, 0.95, 0.48))
        _set_segment(stage, axis_path, center + outward * 0.035, center + outward * 0.32)
        status_position = center + outward * 0.045 + Gf.Vec3d(0.0, 0.0, 0.075)
        _sphere(stage, f"{STATUS_ROOT}/{hole_id}", status_position, 0.016, (0.27, 0.31, 0.36))


def _build_cell(stage) -> None:
    _cube(stage, f"{SCENE_ROOT}/Floor", (0.0, 0.0, -0.05), (2.65, 2.05, 0.05), (0.075, 0.085, 0.10), True)
    _cube(stage, f"{SCENE_ROOT}/SafetyZone", (-0.25, 0.0, 0.008), (1.45, 1.48, 0.008), (0.08, 0.28, 0.30))
    for y in (-1.49, 1.49):
        suffix = f"N{abs(int(y * 100))}" if y < 0 else f"P{int(y * 100)}"
        _cube(stage, f"{SCENE_ROOT}/SafetyLineY_{suffix}", (-0.25, y, 0.016), (1.48, 0.025, 0.012), (0.95, 0.70, 0.08))
    for x in (-1.73, 1.23):
        suffix = f"N{abs(int(x * 100))}" if x < 0 else f"P{int(x * 100)}"
        _cube(stage, f"{SCENE_ROOT}/SafetyLineX_{suffix}", (x, 0.0, 0.016), (0.025, 1.50, 0.012), (0.95, 0.70, 0.08))

    # Mobile-looking cobot pedestal and a compact process cabinet.
    _cube(stage, f"{SCENE_ROOT}/CobotPlatform", (-0.78, 0.0, 0.12), (0.43, 0.46, 0.12), (0.10, 0.12, 0.15), True)
    _cube(stage, f"{SCENE_ROOT}/CobotPlatformAccent", (-0.78, 0.0, 0.245), (0.39, 0.42, 0.018), (0.05, 0.66, 0.72))
    _cube(stage, f"{SCENE_ROOT}/ProcessCabinet", (-1.70, 1.35, 0.43), (0.38, 0.35, 0.43), (0.11, 0.13, 0.17), True)
    _cube(stage, f"{SCENE_ROOT}/ProcessCabinetScreen", (-1.70, 1.00, 0.58), (0.25, 0.012, 0.15), (0.04, 0.55, 0.66))
    _sphere(stage, f"{SCENE_ROOT}/ProcessCabinetBeacon", (-1.70, 1.35, 0.91), 0.045, (0.98, 0.55, 0.06))


def _build_cobot(stage) -> None:
    stage.DefinePrim(COBOT_ROOT, "Xform")
    _cylinder(stage, f"{COBOT_ROOT}/BaseColumn", 0.13, (0.20, 0.23, 0.27))
    _set_segment(stage, f"{COBOT_ROOT}/BaseColumn", (-0.78, 0.0, 0.25), (-0.78, 0.0, 0.53))
    for index in range(4):
        _cylinder(stage, f"{COBOT_ROOT}/Link{index}", 0.055, (0.70, 0.73, 0.76))
    for index in range(5):
        _sphere(stage, f"{COBOT_ROOT}/Joint{index}", (0.0, 0.0, 0.0), 0.085, (0.05, 0.62, 0.66))
    _cylinder(stage, f"{COBOT_ROOT}/CableA", 0.014, (0.025, 0.030, 0.038))
    _cylinder(stage, f"{COBOT_ROOT}/CableB", 0.012, (0.025, 0.030, 0.038))

    stage.DefinePrim(TOOL_ROOT, "Xform")
    for name, radius, color in (
        ("Motor", 0.095, (0.72, 0.075, 0.055)),
        ("Gearbox", 0.070, (0.18, 0.20, 0.23)),
        ("ClampBody", 0.092, (0.94, 0.55, 0.06)),
        ("Nose", 0.035, (0.70, 0.74, 0.78)),
        ("DrillBit", 0.010, (0.18, 0.20, 0.22)),
    ):
        _cylinder(stage, f"{TOOL_ROOT}/{name}", radius, color)
    _cube(stage, f"{TOOL_ROOT}/VisionPod", (0.0, 0.0, 0.0), (0.055, 0.045, 0.035), (0.08, 0.11, 0.15))
    _sphere(stage, f"{TOOL_ROOT}/VisionLens", (0.0, 0.0, 0.0), 0.018, (0.10, 0.65, 0.95))

    frame = hole_frames()["H01"]
    update_cobot_visual(stage, (0.32, 0.0, 1.35), frame.outward)


def update_cobot_visual(stage, tcp, outward, active: bool = False) -> None:
    tcp_v = Gf.Vec3d(*tcp)
    outward_v = Gf.Vec3d(*outward).GetNormalized()
    mount = tcp_v + outward_v * 0.43
    base = Gf.Vec3d(-0.78, 0.0, 0.50)
    shoulder = Gf.Vec3d(-0.62, 0.0, 0.76)
    elbow = Gf.Vec3d(-0.34, mount[1] * 0.42, max(1.34, mount[2] + 0.22))
    wrist = Gf.Vec3d(0.10, mount[1] * 0.76, max(1.16, mount[2] + 0.13))
    joints = (base, shoulder, elbow, wrist, mount)
    for index in range(4):
        _set_segment(stage, f"{COBOT_ROOT}/Link{index}", joints[index], joints[index + 1])
    for index, point in enumerate(joints):
        _set_position(stage, f"{COBOT_ROOT}/Joint{index}", point)
    _set_segment(stage, f"{COBOT_ROOT}/CableA", shoulder + Gf.Vec3d(0, 0, 0.06), elbow + Gf.Vec3d(0, 0, 0.08))
    _set_segment(stage, f"{COBOT_ROOT}/CableB", elbow + Gf.Vec3d(0, 0, 0.08), mount + Gf.Vec3d(0, 0, 0.10))

    _set_segment(stage, f"{TOOL_ROOT}/Motor", tcp_v + outward_v * 0.29, tcp_v + outward_v * 0.48)
    _set_segment(stage, f"{TOOL_ROOT}/Gearbox", tcp_v + outward_v * 0.14, tcp_v + outward_v * 0.31)
    _set_segment(stage, f"{TOOL_ROOT}/ClampBody", tcp_v + outward_v * 0.055, tcp_v + outward_v * 0.15)
    _set_segment(stage, f"{TOOL_ROOT}/Nose", tcp_v, tcp_v + outward_v * 0.065)
    _set_segment(stage, f"{TOOL_ROOT}/DrillBit", tcp_v - outward_v * 0.028, tcp_v + outward_v * 0.085)
    lens_position = tcp_v + outward_v * 0.14 + Gf.Vec3d(0.0, 0.0, 0.10)
    _set_position(stage, f"{TOOL_ROOT}/VisionPod", lens_position)
    _set_position(stage, f"{TOOL_ROOT}/VisionLens", lens_position - outward_v * 0.045)
    _set_color(stage, f"{TOOL_ROOT}/ClampBody", (0.98, 0.72, 0.08) if active else (0.94, 0.55, 0.06))


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
    dome.CreateIntensityAttr(520.0)
    dome.CreateColorAttr(Gf.Vec3f(0.62, 0.72, 0.86))
    key = UsdLux.RectLight.Define(stage, f"{SCENE_ROOT}/KeyLight")
    key.CreateIntensityAttr(2500.0)
    key.CreateWidthAttr(3.0)
    key.CreateHeightAttr(2.0)
    UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(-0.8, -1.6, 3.3))

    _build_cell(stage)
    _build_aircraft_panel(stage)
    _build_cobot(stage)

    if output_path:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output))

    return {
        "scene": SCENE_ROOT,
        "cobot": COBOT_ROOT,
        "tool": TOOL_ROOT,
        "drpe": DRPE_ROOT,
    }
