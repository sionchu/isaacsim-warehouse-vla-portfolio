from __future__ import annotations

from pathlib import Path

from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
NOVA_CARTER_ROS = f"{ASSET_ROOT}/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
WAREHOUSE_ENVIRONMENT = f"{ASSET_ROOT}/Isaac/Environments/Simple_Warehouse/warehouse.usd"
SCENE_ROOT = "/World/WarehouseMission"
ROBOT_PATH = "/World/Nova_Carter_ROS"
SHELF_HEIGHTS = {level: 0.45 + (level - 1) * 0.48 for level in range(1, 6)}
RACK_Y = {"B": 2.6, "C": -2.6}


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


def _cylinder(stage, path: str, position, radius: float, height: float, color):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.XformCommonAPI(cylinder.GetPrim()).SetTranslate(Gf.Vec3d(*position))
    return cylinder.GetPrim()


def build_scene(stage, output_path: str | Path | None = None) -> dict[str, str]:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    existing = stage.GetPrimAtPath(SCENE_ROOT)
    if existing.IsValid():
        stage.RemovePrim(SCENE_ROOT)
    stage.DefinePrim(SCENE_ROOT, "Xform")

    environment = stage.DefinePrim(f"{SCENE_ROOT}/NVIDIAWarehouse", "Xform")
    environment.GetReferences().AddReference(WAREHOUSE_ENVIRONMENT)

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    dome = UsdLux.DomeLight.Define(stage, f"{SCENE_ROOT}/WarehouseLight")
    dome.CreateIntensityAttr(650.0)
    dome.CreateColorAttr(Gf.Vec3f(0.86, 0.91, 1.0))

    _cube(stage, f"{SCENE_ROOT}/Floor", (0.0, 0.0, -0.08), (8.0, 5.0, 0.08), (0.18, 0.20, 0.22), True)
    _cube(stage, f"{SCENE_ROOT}/WallNorth", (0.0, 5.0, 1.25), (8.0, 0.08, 1.25), (0.35, 0.38, 0.42), True)
    _cube(stage, f"{SCENE_ROOT}/WallSouth", (0.0, -5.0, 1.25), (8.0, 0.08, 1.25), (0.35, 0.38, 0.42), True)
    _cube(stage, f"{SCENE_ROOT}/WallWest", (-8.0, 0.0, 1.25), (0.08, 5.0, 1.25), (0.35, 0.38, 0.42), True)

    # A loading zone and B/C destination markings.
    _cube(stage, f"{SCENE_ROOT}/ZoneA", (-6.0, 0.0, 0.012), (1.1, 1.1, 0.012), (0.10, 0.45, 0.95))
    for rack, y in RACK_Y.items():
        color = (0.12, 0.70, 0.32) if rack == "B" else (0.95, 0.48, 0.10)
        _cube(stage, f"{SCENE_ROOT}/Zone{rack}", (4.0, y, 0.012), (0.8, 0.8, 0.012), color)
        _build_rack(stage, rack, y, color)

    # Static pallet islands used by the A* safety planner.
    _cube(stage, f"{SCENE_ROOT}/ObstacleWest", (0.35, 0.0, 0.35), (0.85, 1.0, 0.35), (0.50, 0.28, 0.12), True)
    _cube(stage, f"{SCENE_ROOT}/ObstacleEast", (2.3, 1.25, 0.35), (0.5, 0.75, 0.35), (0.50, 0.28, 0.12), True)

    _create_robot_and_lift(stage)

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output_path))

    return {
        "environment": f"{SCENE_ROOT}/NVIDIAWarehouse",
        "robot": ROBOT_PATH,
        "cargo": f"{ROBOT_PATH}/PortfolioLift/ActiveCargo",
        "tray": f"{ROBOT_PATH}/PortfolioLift/Tray",
    }


def _build_rack(stage, rack: str, y: float, color) -> None:
    root = f"{SCENE_ROOT}/Rack{rack}"
    stage.DefinePrim(root, "Xform")
    for support_index, support_y in enumerate((y - 0.9, y + 0.9)):
        _cube(stage, f"{root}/Support{support_index}", (5.8, support_y, 1.35), (0.06, 0.06, 1.35), color, True)
    for level, z in SHELF_HEIGHTS.items():
        _cube(stage, f"{root}/Shelf{level}", (5.8, y, z), (0.55, 0.95, 0.035), color, True)
        _cube(stage, f"{root}/SlotMarker{level}", (5.30, y, z + 0.16), (0.02, 0.34, 0.13), (0.15, 0.15, 0.15))


def _create_robot_and_lift(stage) -> None:
    robot = stage.GetPrimAtPath(ROBOT_PATH)
    if not robot.IsValid():
        robot = stage.DefinePrim(ROBOT_PATH, "Xform")
        robot.GetReferences().AddReference(NOVA_CARTER_ROS)
    UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(-6.0, 0.0, 0.0))

    lift_root = f"{ROBOT_PATH}/PortfolioLift"
    stage.DefinePrim(lift_root, "Xform")
    _cube(stage, f"{lift_root}/MastLeft", (0.0, -0.28, 1.25), (0.05, 0.05, 1.05), (0.18, 0.20, 0.24))
    _cube(stage, f"{lift_root}/MastRight", (0.0, 0.28, 1.25), (0.05, 0.05, 1.05), (0.18, 0.20, 0.24))
    _cube(stage, f"{lift_root}/Tray", (0.0, 0.0, 0.65), (0.38, 0.42, 0.035), (0.10, 0.55, 0.85))
    _cube(stage, f"{lift_root}/ActiveCargo", (0.0, 0.0, 0.84), (0.26, 0.30, 0.16), (0.76, 0.52, 0.20))
    _cylinder(stage, f"{lift_root}/Beacon", (0.0, 0.0, 2.42), 0.08, 0.12, (0.95, 0.15, 0.10))


def slot_box_position(slot_id: str) -> tuple[float, float, float]:
    rack = slot_id[0]
    level = int(slot_id[1:])
    return 5.45, RACK_Y[rack], SHELF_HEIGHTS[level] + 0.19


def approach_position(slot_id: str) -> tuple[float, float]:
    return 4.0, RACK_Y[slot_id[0]]
