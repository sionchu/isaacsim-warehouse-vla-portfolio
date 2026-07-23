from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path

from isaacsim import SimulationApp

ROS_LIB = r"C:\isaacsim\exts\isaacsim.ros2.core\jazzy\lib"
os.environ["ROS_DISTRO"] = "jazzy"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
if ROS_LIB.lower() not in os.environ.get("PATH", "").lower().split(";"):
    os.environ["PATH"] = os.environ.get("PATH", "") + ";" + ROS_LIB

simulation_app = SimulationApp({"headless": True})

import carb.settings  # noqa: E402
import omni.ext  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/internal_lib_fallback",
        True,
    )
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_path = ROOT / "isaacsim_exts" / "aero.drill.vla"
    manager.add_path(str(extension_path), omni.ext.ExtensionPathType.DIRECT_PATH)
    manager.set_extension_enabled_immediate("aero.drill.vla", True)
    for _ in range(120):
        simulation_app.update()
        if not is_stage_loading():
            continue
    stage = omni.usd.get_context().get_stage()
    assert stage is not None
    assert stage.GetPrimAtPath("/World/AeroDrillVLA").IsValid()
    assert stage.GetPrimAtPath("/World/AeroDrillVLA/UR10e/wrist_3_link").IsValid()
    assert stage.GetPrimAtPath(
        "/World/AeroDrillVLA/UR10e/joints/shoulder_pan_joint"
    ).IsA(UsdPhysics.RevoluteJoint)
    assert stage.GetPrimAtPath(
        "/World/AeroDrillVLA/AeroDrillTool/TCP"
    ).IsValid()
    assert stage.GetPrimAtPath(
        "/World/AeroDrillVLA/CoordinateFrames/UR_Base"
    ).IsValid()
    assert not stage.GetPrimAtPath("/World/AeroDrillVLA/Cobot/Link0").IsValid()
    collision_count = sum(
        1
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    )
    assert collision_count >= 10
    result = {
        "status": "PASS",
        "extension": "aero.drill.vla",
        "scene_prim": "/World/AeroDrillVLA",
        "robot": "official UR10e asset",
        "revolute_joints": 6,
        "collision_shapes": collision_count,
    }
    result_path = ROOT / "recordings" / "raw" / "aero_drill_extension_result.json"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("aero_drill_extension=PASS", flush=True)
    manager.set_extension_enabled_immediate("aero.drill.vla", False)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        error_path = ROOT / "recordings" / "raw" / "aero_drill_extension_error.txt"
        error_path.parent.mkdir(parents=True, exist_ok=True)
        error_path.write_text(traceback.format_exc(), encoding="utf-8")
        raise
    finally:
        simulation_app.close()
