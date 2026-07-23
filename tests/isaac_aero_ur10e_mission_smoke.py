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
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "isaacsim_exts" / "aero.drill.vla"))

from aero_drill_vla.mission_controller import AeroDrillMissionController  # noqa: E402
from aero_drill_vla.scene_builder import build_scene  # noqa: E402


def main() -> None:
    result_path = ROOT / "recordings" / "raw" / "aero_drill_ur10e_mission_result.json"
    error_path = ROOT / "recordings" / "raw" / "aero_drill_ur10e_mission_error.txt"
    for old_result in (result_path, error_path):
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

    statuses: list[str] = []
    controller = AeroDrillMissionController(
        stage,
        ROOT / "models" / "aero_drill_vla.pt",
        ROOT / "recordings" / "aero_drill_ur10e_mission.jsonl",
        statuses.append,
    )
    controller.cycle_speed = 1.6
    app_utils.play()
    for _ in range(180):
        controller.update(1.0 / 60.0)
        simulation_app.update()
        if controller.robot_ready:
            break
    assert controller.robot_ready, controller.robot.last_error

    controller.start_selected("H01", "process hole H01")
    max_error_by_state: dict[str, float] = {}
    frames = 0
    for frames in range(7200):
        controller.update(1.0 / 60.0)
        simulation_app.update()
        max_error_by_state[controller.state] = max(
            max_error_by_state.get(controller.state, 0.0),
            controller.tcp_error_mm,
        )
        if controller.state == "IDLE" and controller.completed["H01"]:
            break

    timeout_messages = [message for message in statuses if "motion timeout gate" in message]
    assert controller.completed["H01"], f"H01 did not complete after {frames} frames"
    assert not timeout_messages, timeout_messages
    assert controller.tcp_error_mm < 40.0, controller.tcp_error_mm

    result = {
        "status": "PASS",
        "hole": "H01",
        "frames": frames + 1,
        "seconds": round((frames + 1) / 60.0, 3),
        "final_tcp_error_mm": round(controller.tcp_error_mm, 3),
        "final_clearance_mm": round(controller.clearance_mm, 3),
        "max_error_by_state_mm": {
            key: round(value, 3)
            for key, value in max_error_by_state.items()
        },
        "joint_positions_deg": [
            round(float(row["position_deg"]), 3)
            for row in controller.joint_rows()
        ],
        "collision_shapes": controller.robot.collision_world_count,
        "status_tail": statuses[-8:],
    }
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        error_path = ROOT / "recordings" / "raw" / "aero_drill_ur10e_mission_error.txt"
        error_path.parent.mkdir(parents=True, exist_ok=True)
        error_path.write_text(traceback.format_exc(), encoding="utf-8")
        raise
    finally:
        simulation_app.close()
