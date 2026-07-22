from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "isaacsim_exts" / "warehouse.mission"))

from warehouse_mission.mission_controller import MissionController  # noqa: E402
from warehouse_mission.scene_builder import build_scene  # noqa: E402


def run_to_idle(controller: MissionController, max_steps: int = 400) -> None:
    for _ in range(max_steps):
        controller.update(0.1)
        if controller.state == "IDLE":
            return
    raise AssertionError(f"mission did not finish: state={controller.state}")


def main() -> None:
    stage = Usd.Stage.CreateInMemory()
    build_scene(stage)
    controller = MissionController(
        stage,
        ROOT / "models" / "warehouse_vla.pt",
        ROOT / "recordings" / "smoke_missions.jsonl",
    )
    assert controller.policy.mode == "trained VLA-lite"

    manual = controller.start(
        "B \uad6c\uc5ed 3\uce35\uc5d0 \ubc15\uc2a4\ub97c \ubcf4\uad00\ud574", requested_slot="B3"
    )
    assert manual.slot_id == "B3"
    run_to_idle(controller)
    assert controller.occupancy["B3"]

    automatic = controller.start("\ube44\uc5b4 \uc788\ub294 \uac04\ubc18\uc5d0 \uc790\ub3d9 \ubcf4\uad00\ud574")
    assert automatic.slot_id != "B3"
    run_to_idle(controller)
    assert controller.occupancy[automatic.slot_id]
    print(
        "warehouse_smoke=PASS "
        f"manual={manual.slot_id} automatic={automatic.slot_id} policy={controller.policy.mode}"
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
