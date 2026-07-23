from __future__ import annotations

import json
import sys
import traceback
from pathlib import Path

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot_motion.cumotion", True
)

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "isaacsim_exts" / "aero.drill.vla"))

from aero_drill_vla.robot_controller import JOINT_NAMES, UR10eMotionController  # noqa: E402
from aero_drill_vla.scene_builder import build_scene, hole_frames  # noqa: E402


def main() -> None:
    for old_result in (
        ROOT / "recordings" / "raw" / "aero_drill_ur10e_motion_result.json",
        ROOT / "recordings" / "raw" / "aero_drill_ur10e_motion_error.txt",
    ):
        if old_result.exists():
            old_result.unlink()
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cuda")
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    build_scene(stage)
    while is_stage_loading():
        simulation_app.update()
    for _ in range(80):
        simulation_app.update()

    app_utils.play()
    for _ in range(20):
        simulation_app.update()

    robot = UR10eMotionController(stage)
    for _ in range(120):
        if robot.ensure_initialized():
            break
        simulation_app.update()
    assert robot.ready, robot.last_error
    assert tuple(robot.articulation.dof_names) == JOINT_NAMES
    assert len(robot.articulation.link_names) == 7
    assert robot.collision_enabled
    assert robot.collision_world_count >= 10

    frame = hole_frames()["H03"]
    center = np.asarray(frame.center, dtype=np.float32)
    outward = np.asarray(frame.outward, dtype=np.float32)
    target_tcp = center + outward * 0.14
    for _ in range(1200):
        robot.command_tcp(target_tcp, outward, 1.0 / 60.0)
        simulation_app.update()
        if robot.tcp_error_mm < 25.0:
            break

    assert robot.tcp_error_mm < 45.0, f"TCP error remained {robot.tcp_error_mm:.1f} mm"
    joint_rows = robot.joint_rows()
    assert len(joint_rows) == 6
    assert all(np.isfinite(float(row["position_deg"])) for row in joint_rows)
    assert all(
        float(row["lower_deg"]) <= float(row["position_deg"]) <= float(row["upper_deg"])
        for row in joint_rows
    )

    result = {
        "status": "PASS",
        "robot": "NVIDIA official UR10e USD",
        "motion": "cuMotion RMPflow",
        "dof_names": list(JOINT_NAMES),
        "link_names": robot.articulation.link_names,
        "collision_shapes": robot.collision_world_count,
        "target_tcp_m": target_tcp.round(5).tolist(),
        "actual_tcp_m": robot.current_tcp.round(5).tolist(),
        "tcp_error_mm": round(robot.tcp_error_mm, 3),
        "joint_positions_deg": [
            round(float(row["position_deg"]), 3)
            for row in joint_rows
        ],
    }
    result_path = ROOT / "recordings" / "raw" / "aero_drill_ur10e_motion_result.json"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        error_path = ROOT / "recordings" / "raw" / "aero_drill_ur10e_motion_error.txt"
        error_path.parent.mkdir(parents=True, exist_ok=True)
        error_path.write_text(traceback.format_exc(), encoding="utf-8")
        raise
    finally:
        simulation_app.close()
