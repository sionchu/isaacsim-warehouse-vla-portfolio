from __future__ import annotations

import sys
import json
import traceback
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "isaacsim_exts" / "aero.drill.vla"))

from aero_drill_vla.hole_policy import HOLE_IDS  # noqa: E402
from aero_drill_vla.mission_controller import AeroDrillMissionController  # noqa: E402
from aero_drill_vla.scene_builder import build_scene  # noqa: E402


def main() -> None:
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    build_scene(stage)
    controller = AeroDrillMissionController(
        stage,
        ROOT / "models" / "aero_drill_vla.pt",
        ROOT / "recordings" / "aero_drill_smoke.jsonl",
        enable_robot_motion=False,
    )
    assert controller.policy.mode == "trained aero VLA-lite"
    controller.cycle_speed = 10.0
    first = controller.start_selected("H03", "process hole H03")
    assert first.hole_id == "H03"
    for _ in range(300):
        controller.update(0.1)
        if controller.state == "IDLE":
            break
    assert controller.completed["H03"]

    decision = controller.start_sequence("process the next pending DRPE hole")
    assert decision.hole_id in HOLE_IDS
    for _ in range(4000):
        controller.update(0.1)
        if controller.state == "IDLE":
            break
    assert controller.completed_count == 10
    assert controller.cycle_order[-9:] == [hole for hole in HOLE_IDS if hole != "H03"]
    result = {
        "status": "PASS",
        "completed": controller.completed_count,
        "policy": controller.policy.mode,
        "last_quality": round(controller.last_quality, 1),
        "sequence_after_manual_h03": controller.cycle_order,
    }
    result_path = ROOT / "recordings" / "raw" / "aero_drill_smoke_result.json"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(
        "aero_drill_smoke=PASS "
        f"completed={controller.completed_count} policy={controller.policy.mode} "
        f"last_quality={controller.last_quality:.1f}",
        flush=True,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception:
        error_path = ROOT / "recordings" / "raw" / "aero_drill_smoke_error.txt"
        error_path.parent.mkdir(parents=True, exist_ok=True)
        error_path.write_text(traceback.format_exc(), encoding="utf-8")
        raise
    finally:
        simulation_app.close()
