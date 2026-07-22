from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from isaacsim import SimulationApp


def parse_args():
    parser = argparse.ArgumentParser(description="Render an automated warehouse mission trial.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--preview", action="store_true")
    return parser.parse_known_args()


ARGS, ISAAC_ARGS = parse_args()
sys.argv = [sys.argv[0], *ISAAC_ARGS]
simulation_app = SimulationApp(
    {"headless": True, "renderer": "RaytracedLighting", "width": 960, "height": 540}
)

import carb.settings  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading  # noqa: E402
from omni.replicator.core.functional import write_image  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "isaacsim_exts" / "warehouse.mission"))

from warehouse_mission.mission_controller import MissionController  # noqa: E402
from warehouse_mission.scene_builder import build_scene  # noqa: E402


def prepare_output(output_dir: Path) -> None:
    output_dir = output_dir.resolve()
    allowed_root = (ROOT / "recordings" / "raw").resolve()
    if output_dir != allowed_root and allowed_root not in output_dir.parents:
        raise RuntimeError(f"Raw capture must stay below {allowed_root}")
    output_dir.mkdir(parents=True, exist_ok=True)
    for old_frame in output_dir.glob("frame_*.png"):
        old_frame.unlink()
    telemetry = output_dir / "telemetry.json"
    if telemetry.exists():
        telemetry.unlink()


def wait_for_stage() -> None:
    simulation_app.update()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    for _ in range(12):
        simulation_app.update()


def record() -> None:
    output_dir = ARGS.output_dir.resolve()
    prepare_output(output_dir)
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    build_scene(stage)
    wait_for_stage()

    latest_status = {"text": "System ready"}

    def status_callback(message: str) -> None:
        latest_status["text"] = message

    controller = MissionController(
        stage,
        ROOT / "models" / "warehouse_vla.pt",
        ROOT / "recordings" / "demo_missions.jsonl",
        status_callback,
    )
    controller.travel_speed = 3.0

    rep.orchestrator.set_capture_on_play(False)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)
    camera = rep.functional.create.camera(
        position=(0.0, -4.0, 5.6),
        look_at=(0.0, 0.0, 0.5),
        focal_length=6.5,
        clipping_range=(0.1, 1000.0),
        parent="/World",
        name="PortfolioRecordingCamera",
    )
    render_product = rep.create.render_product(camera, (960, 540), name="WarehouseTrialRender")
    rgb = rep.annotators.get("rgb")
    rgb.attach(render_product)

    # Compile shaders and discard the warm-up image.
    rep.orchestrator.step(rt_subframes=4)

    if ARGS.preview:
        write_image(path=str(output_dir / "frame_0000.png"), data=rgb.get_data())
        telemetry = [snapshot(controller, latest_status["text"], "Preview", "")]
        (output_dir / "telemetry.json").write_text(
            json.dumps(telemetry, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"preview={output_dir / 'frame_0000.png'}", flush=True)
        cleanup(rgb, render_product)
        return

    telemetry = []
    phase = "preroll"
    phase_frame = 0
    mission_label = "System check"
    selected_slot = ""
    max_frames = 260
    dt = 0.12

    for frame_index in range(max_frames):
        if phase == "preroll" and frame_index >= 12:
            decision = controller.start(
                "C \uad6c\uc5ed 5\uce35\uc5d0 \ubc15\uc2a4\ub97c \ubcf4\uad00\ud574", requested_slot="C5"
            )
            selected_slot = decision.slot_id
            mission_label = "Manual mission: C5"
            phase = "manual"
        elif phase == "manual" and controller.state == "IDLE" and controller.occupancy["C5"]:
            phase = "between"
            phase_frame = frame_index
            latest_status["text"] = "Manual mission verified"
        elif phase == "between" and frame_index - phase_frame >= 12:
            decision = controller.start("\ube44\uc5b4 \uc788\ub294 \uac04\ubc18\uc5d0 \uc790\ub3d9 \ubcf4\uad00\ud574")
            selected_slot = decision.slot_id
            mission_label = f"Automatic mission: {decision.slot_id}"
            phase = "automatic"
        elif (
            phase == "automatic"
            and controller.state == "IDLE"
            and sum(controller.occupancy.values()) >= 2
        ):
            phase = "postroll"
            phase_frame = frame_index
            latest_status["text"] = "Trial complete: two boxes stored"

        controller.update(dt)
        rep.orchestrator.step(rt_subframes=1)
        frame_path = output_dir / f"frame_{frame_index:04d}.png"
        write_image(path=str(frame_path), data=rgb.get_data())
        telemetry.append(snapshot(controller, latest_status["text"], mission_label, selected_slot))

        if frame_index % ARGS.fps == 0:
            print(
                f"capture frame={frame_index:03d} state={controller.state} "
                f"position=({controller.current_position.x:.2f},{controller.current_position.y:.2f})",
                flush=True,
            )
        if phase == "postroll" and frame_index - phase_frame >= 20:
            break

    (output_dir / "telemetry.json").write_text(
        json.dumps(telemetry, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"frames={len(telemetry)}", flush=True)
    print(f"telemetry={output_dir / 'telemetry.json'}", flush=True)
    cleanup(rgb, render_product)


def snapshot(controller, status: str, mission: str, selected_slot: str) -> dict:
    return {
        "state": controller.state,
        "status": status,
        "mission": mission,
        "selected_slot": selected_slot,
        "position": [controller.current_position.x, controller.current_position.y],
        "route": [[point.x, point.y] for point in controller.route],
        "occupancy": dict(controller.occupancy),
        "policy": controller.policy.mode,
    }


def cleanup(rgb, render_product) -> None:
    rep.orchestrator.wait_until_complete()
    rgb.detach()
    render_product.destroy()


try:
    record()
finally:
    simulation_app.close()
