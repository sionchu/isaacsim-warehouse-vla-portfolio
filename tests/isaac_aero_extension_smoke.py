from __future__ import annotations

import json
import sys
import traceback
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.ext  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_path = ROOT / "isaacsim_exts" / "aero.drill.vla"
    manager.add_path(str(extension_path), omni.ext.ExtensionPathType.DIRECT_PATH)
    manager.set_extension_enabled_immediate("aero.drill.vla", True)
    for _ in range(20):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    assert stage is not None
    assert stage.GetPrimAtPath("/World/AeroDrillVLA").IsValid()
    result = {
        "status": "PASS",
        "extension": "aero.drill.vla",
        "scene_prim": "/World/AeroDrillVLA",
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
