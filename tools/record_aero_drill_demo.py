from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path

from isaacsim import SimulationApp


def parse_args():
    parser = argparse.ArgumentParser(description="Render the aerospace DRPE drilling trial.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--max-holes", type=int, default=3)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--ros", action="store_true")
    return parser.parse_known_args()


ARGS, ISAAC_ARGS = parse_args()
sys.argv = [sys.argv[0], *ISAAC_ARGS]
simulation_app = SimulationApp(
    {"headless": True, "renderer": "RaytracedLighting", "width": 960, "height": 540}
)

import carb.settings  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from omni.replicator.core.functional import write_image  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot_motion.cumotion", True
)
if ARGS.ros:
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/internal_lib_fallback",
        True,
    )
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
        "isaacsim.ros2.bridge", True
    )
    for _ in range(8):
        simulation_app.update()

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "isaacsim_exts" / "aero.drill.vla"))

from aero_drill_vla.mission_controller import AeroDrillMissionController  # noqa: E402
from aero_drill_vla.hole_policy import HOLE_IDS  # noqa: E402
from aero_drill_vla.ros_bridge import AeroDrillRosBridge  # noqa: E402
from aero_drill_vla.scene_builder import build_scene  # noqa: E402


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
    for _ in range(10):
        simulation_app.update()


def snapshot(
    controller: AeroDrillMissionController,
    status: str,
    ros_bridge: AeroDrillRosBridge | None = None,
) -> dict:
    active = controller.active_hole
    frame = controller.frames.get(active)
    return {
        "state": controller.state,
        "status": status,
        "active_hole": active,
        "strategy": controller.active_strategy,
        "completed": dict(controller.completed),
        "completed_count": controller.completed_count,
        "force_n": controller.current_force_n,
        "spindle_rpm": controller.spindle_rpm,
        "feed_mm_s": controller.feed_mm_s,
        "quality": controller.last_quality,
        "policy": controller.policy.mode,
        "position_error_mm": frame.position_error_mm if frame else 0.0,
        "normal_error_deg": frame.normal_error_deg if frame else 0.0,
        "material_stack": frame.material_stack if frame else "--",
        "robot": controller.robot_mode,
        "tcp_error_mm": controller.tcp_error_mm,
        "clearance_mm": controller.clearance_mm,
        "collision": controller.collision_summary,
        "tcp_pose": controller.tcp_pose_text,
        "joints": controller.joint_rows(),
        "ros_enabled": bool(ros_bridge and ros_bridge.available),
        "ros_command_count": ros_bridge.command_count if ros_bridge else 0,
        "ros_publish_count": ros_bridge.publish_count if ros_bridge else 0,
        "ros_terminal": list(ros_bridge.events) if ros_bridge else [],
    }


def cleanup(rgb, render_product) -> None:
    rep.orchestrator.wait_until_complete()
    rgb.detach()
    render_product.destroy()


def capture_rgb(rgb, *, subframes: int = 2):
    for _ in range(20):
        rep.orchestrator.step(
            rt_subframes=subframes,
            delta_time=0.0,
            pause_timeline=False,
        )
        data = rgb.get_data()
        if getattr(data, "size", 0) > 0:
            return data
        simulation_app.update()
    raise RuntimeError("Replicator RGB buffer remained empty after 20 synchronization frames")


def record() -> None:
    output_dir = ARGS.output_dir.resolve()
    prepare_output(output_dir)
    error_path = ROOT / "recordings" / "raw" / "aero_drill_error.txt"
    if error_path.exists():
        error_path.unlink()
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    build_scene(stage)
    wait_for_stage()

    rep.orchestrator.set_capture_on_play(False)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)
    camera = rep.functional.create.camera(
        position=(-1.95, -2.15, 1.75),
        look_at=(0.42, 0.0, 0.62),
        focal_length=30.0,
        clipping_range=(0.1, 1000.0),
        parent="/World",
        name="AeroDrillRecordingCamera",
    )
    render_product = rep.create.render_product(camera, (960, 540), name="AeroDrillTrialRender")
    rgb = rep.annotators.get("rgb")
    rgb.attach(render_product)
    capture_rgb(rgb, subframes=6)
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cuda")

    latest_status = {"text": "System ready"}

    def status_callback(message: str) -> None:
        latest_status["text"] = message

    controller = AeroDrillMissionController(
        stage,
        ROOT / "models" / "aero_drill_vla.pt",
        ROOT / "recordings" / "aero_drill_demo_events.jsonl",
        status_callback,
    )
    controller.cycle_speed = 1.6
    ros_bridge = None

    def ros_command(payload: dict) -> dict:
        action = str(payload.get("action", "")).upper()
        instruction = str(payload.get("instruction", "")).strip()
        try:
            if action == "RUN_HOLE":
                hole = str(payload.get("hole", "")).upper()
                if hole not in HOLE_IDS:
                    raise ValueError(f"unknown hole: {hole}")
                decision = controller.start_selected(
                    hole,
                    instruction or f"ROS 2 command: process {hole}",
                )
                message = f"mission started: {decision.hole_id} {decision.strategy}"
            elif action == "RUN_BATCH":
                decision = controller.start_sequence(
                    instruction or "ROS 2 command: process H01-H10"
                )
                message = f"batch started: {decision.hole_id}"
            elif action == "PAUSE":
                controller.pause()
                message = f"pause toggled: {controller.paused}"
            elif action == "RESET":
                controller.reset()
                message = "batch reset"
            elif action == "PING":
                message = "bridge online"
            else:
                raise ValueError(f"unsupported action: {action}")
            latest_status["text"] = f"ROS 2 {message}"
            return {
                "accepted": True,
                "action": action,
                "hole": str(payload.get("hole", "--")).upper(),
                "message": message,
            }
        except Exception as error:
            latest_status["text"] = f"ROS 2 command rejected: {error}"
            return {
                "accepted": False,
                "action": action,
                "hole": str(payload.get("hole", "--")).upper(),
                "message": str(error),
            }

    if ARGS.ros:
        ros_bridge = AeroDrillRosBridge(ros_command)
        if not ros_bridge.available:
            raise RuntimeError(f"ROS 2 bridge unavailable: {ros_bridge.error}")
    app_utils.play()
    for _ in range(180):
        controller.update(1.0 / 60.0)
        if ros_bridge:
            ros_bridge.tick(1.0 / 60.0, controller, latest_status["text"])
        simulation_app.update()
        if controller.robot_ready:
            break
    if not controller.robot_ready:
        raise RuntimeError(controller.robot.last_error if controller.robot else "UR10e unavailable")

    if ARGS.preview:
        controller.start_selected("H03", "process hole H03")
        for _ in range(720):
            controller.update(1.0 / 60.0)
            simulation_app.update()
            if controller.state in {"SEARCH", "DOCK", "CLAMP"}:
                break
        frame_data = capture_rgb(rgb, subframes=4)
        write_image(path=str(output_dir / "frame_0000.png"), data=frame_data)
        (output_dir / "telemetry.json").write_text(
            json.dumps(
                [snapshot(controller, latest_status["text"], ros_bridge)],
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"preview={output_dir / 'frame_0000.png'}", flush=True)
        cleanup(rgb, render_product)
        return

    telemetry = []
    max_holes = min(max(int(ARGS.max_holes), 1), len(HOLE_IDS))
    selected_holes = list(HOLE_IDS[:max_holes])
    next_hole_index = 0
    max_frames = max(300, max_holes * 180 + 120)
    postroll_frame = None
    for frame_index in range(max_frames):
        if (
            not ARGS.ros
            and
            frame_index >= 10
            and controller.state == "IDLE"
            and next_hole_index < len(selected_holes)
        ):
            hole = selected_holes[next_hole_index]
            controller.start_selected(hole, f"process hole {hole}")
            next_hole_index += 1
        for _ in range(4):
            controller.update(1.0 / 60.0)
            if ros_bridge:
                ros_bridge.tick(1.0 / 60.0, controller, latest_status["text"])
            simulation_app.update()
        frame_data = capture_rgb(rgb, subframes=1)
        frame_path = output_dir / f"frame_{frame_index:04d}.png"
        write_image(path=str(frame_path), data=frame_data)
        telemetry.append(snapshot(controller, latest_status["text"], ros_bridge))
        if frame_index % ARGS.fps == 0:
            print(
                f"capture frame={frame_index:03d} state={controller.state} "
                f"hole={controller.active_hole} complete={controller.completed_count}/10",
                flush=True,
            )
        if controller.completed_count == max_holes and controller.state == "IDLE":
            postroll_frame = postroll_frame or frame_index
            if frame_index - postroll_frame >= 22:
                break

    (output_dir / "telemetry.json").write_text(
        json.dumps(telemetry, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"frames={len(telemetry)}", flush=True)
    cleanup(rgb, render_product)
    if ros_bridge:
        ros_bridge.close()


try:
    record()
except Exception:
    error_text = traceback.format_exc()
    print(error_text, flush=True)
    error_path = ROOT / "recordings" / "raw" / "aero_drill_error.txt"
    error_path.parent.mkdir(parents=True, exist_ok=True)
    error_path.write_text(error_text, encoding="utf-8")
    raise
finally:
    simulation_app.close()
