from __future__ import annotations

from pathlib import Path

import carb.settings
import omni.ext
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd

from .hole_policy import HOLE_IDS
from .mission_controller import AeroDrillMissionController
from .ros_bridge import AeroDrillRosBridge
from .scene_builder import (
    build_scene,
    set_centerlines_visible,
    set_frames_visible,
)

PORTFOLIO_ROOT = Path(__file__).resolve().parents[3]


class AeroDrillVLAExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        self._ext_id = ext_id
        self._window = ui.Window("Aero Drill VLA Control", width=580, height=920)
        self._controller: AeroDrillMissionController | None = None
        self._ros_bridge: AeroDrillRosBridge | None = None
        self._status_text = "Ready"
        self._centerlines_visible = True
        self._frames_visible = True
        self._colliders_visible = False
        self._update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update, name="aero_drill_vla_update")
        )
        self._build_ui()
        self._create_scene()

    def on_shutdown(self) -> None:
        if self._ros_bridge is not None:
            self._ros_bridge.close()
        self._ros_bridge = None
        self._update_subscription = None
        self._controller = None
        self._window = None

    def _build_ui(self) -> None:
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, height=0):
                    ui.Label("AEROSPACE DRILLING VLA", style={"font_size": 22})
                    ui.Label(
                        "Official UR10e | cuMotion RMPflow | DRPE docking",
                        style={"color": 0xFF7FE6F3},
                    )
                    ui.Label(
                        "Generic R-eVo-inspired tool - not OEM geometry",
                        style={"color": 0xFF9AA7B4},
                    )
                    ui.Separator()

                    with ui.HStack(height=30):
                        ui.Label("Selected hole", width=115)
                        self._hole_combo = ui.ComboBox(0, *HOLE_IDS)
                    with ui.HStack(height=30):
                        ui.Label("Instruction", width=115)
                        self._instruction = ui.StringField()
                        self._instruction.model.set_value("process the next pending DRPE hole")

                    with ui.HStack(height=36, spacing=6):
                        ui.Button("Run Selected Hole", clicked_fn=self._run_selected)
                        ui.Button("Run H01-H10 Batch", clicked_fn=self._run_sequence)
                    with ui.HStack(height=36, spacing=6):
                        ui.Button("Pause / Resume", clicked_fn=self._pause_task)
                        ui.Button("Reset Batch", clicked_fn=self._reset_batch)

                    ui.Separator()
                    ui.Label("HOLE QUALITY MAP")
                    self._hole_status_label = ui.Label("", word_wrap=True)

                    ui.Separator()
                    ui.Label("LIVE PROCESS TELEMETRY")
                    self._ros_label = ui.Label("ROS 2: initializing", word_wrap=True)
                    self._active_label = ui.Label("Active hole: --")
                    self._state_label = ui.Label("State: IDLE")
                    self._strategy_label = ui.Label("Strategy: --")
                    self._force_label = ui.Label("Axial force: 0.0 N")
                    self._spindle_label = ui.Label("Spindle: 0 rpm | Feed: 0.0 mm/s")
                    self._quality_label = ui.Label("Last quality: --")
                    self._progress_label = ui.Label("Batch progress: 0 / 10")

                    ui.Separator()
                    ui.Label("ROBOT / COORDINATE FRAMES")
                    self._robot_mode_label = ui.Label("Robot: UR10e initializing", word_wrap=True)
                    self._frame_label = ui.Label(
                        "Frames: W = B | TCP Z+ = drilling direction | Hole Z+ = outward",
                        word_wrap=True,
                    )
                    self._tcp_pose_label = ui.Label("TCP W: --", word_wrap=True)
                    self._ik_label = ui.Label("IK error: --")
                    self._collision_label = ui.Label("Collision: waiting", word_wrap=True)

                    ui.Separator()
                    ui.Label("UR10e SIX-AXIS JOINT STATE")
                    self._joint_labels = []
                    for index in range(6):
                        label = ui.Label(f"J{index + 1}: waiting", style={"font_size": 13})
                        self._joint_labels.append(label)

                    ui.Separator()
                    with ui.HStack(height=36, spacing=6):
                        ui.Button("Toggle Centerlines", clicked_fn=self._toggle_centerlines)
                        ui.Button("Toggle Frames", clicked_fn=self._toggle_frames)
                        ui.Button("Toggle Colliders", clicked_fn=self._toggle_colliders)
                    with ui.HStack(height=36, spacing=6):
                        ui.Button("Rebuild Scene", clicked_fn=self._create_scene)
                        ui.Button("Save USD", clicked_fn=self._save_scene)
                    with ui.HStack(height=36, spacing=6):
                        ui.Button("Simulation Play", clicked_fn=self._play_simulation)
                        ui.Button("Simulation Pause", clicked_fn=self._pause_simulation)

                    self._policy_label = ui.Label("Policy: initializing")
                    self._status_label = ui.Label("Ready", word_wrap=True)
                    ui.Spacer(height=8)
                    ui.Label("Task policy: language + 10-hole visual state", style={"color": 0xFF9AA7B4})
                    ui.Label("Motion: fixed-link UR10e + collision-aware RMPflow", style={"color": 0xFF9AA7B4})
                    ui.Label("Safety gate: Direct / Vision Refine / Spiral", style={"color": 0xFF9AA7B4})
                    ui.Label("Log: recordings/aero_drill_events.jsonl", style={"color": 0xFF9AA7B4})

    def _create_scene(self) -> None:
        omni.timeline.get_timeline_interface().pause()
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        output = PORTFOLIO_ROOT / "scenes" / "aero_drill_vla.usda"
        build_scene(stage, output)
        self._controller = AeroDrillMissionController(
            stage,
            PORTFOLIO_ROOT / "models" / "aero_drill_vla.pt",
            PORTFOLIO_ROOT / "recordings" / "aero_drill_events.jsonl",
            self._set_status,
        )
        if self._ros_bridge is not None:
            self._ros_bridge.close()
        self._ros_bridge = AeroDrillRosBridge(self._on_ros_command)
        self._centerlines_visible = True
        self._frames_visible = True
        self._policy_label.text = f"Policy: {self._controller.policy.mode}"
        self._refresh_ui()
        ros_state = (
            "ROS 2 bridge ready"
            if self._ros_bridge.available
            else f"ROS 2 unavailable: {self._ros_bridge.error}"
        )
        self._set_status(f"Scene ready | {ros_state} | Press Run or publish a mission")

    def _on_ros_command(self, payload: dict) -> dict:
        if self._controller is None:
            return {"accepted": False, "message": "controller is not ready"}
        action = str(payload.get("action", "")).upper()
        instruction = str(payload.get("instruction", "")).strip()
        try:
            if action == "RUN_HOLE":
                hole = str(payload.get("hole", "")).upper()
                if hole not in HOLE_IDS:
                    raise ValueError(f"unknown hole: {hole}")
                omni.timeline.get_timeline_interface().play()
                decision = self._controller.start_selected(
                    hole,
                    instruction or f"ROS 2 command: process {hole}",
                )
                message = (
                    f"ROS accepted: {decision.hole_id} | "
                    f"{decision.strategy} | {decision.source}"
                )
            elif action == "RUN_BATCH":
                omni.timeline.get_timeline_interface().play()
                decision = self._controller.start_sequence(
                    instruction or "ROS 2 command: process H01-H10 in sequence"
                )
                message = f"ROS batch accepted: starts {decision.hole_id}"
            elif action == "PAUSE":
                self._controller.pause()
                message = f"ROS pause toggled: paused={self._controller.paused}"
            elif action == "RESET":
                self._controller.reset()
                message = "ROS reset accepted"
            elif action == "PING":
                message = "ROS bridge online"
            else:
                raise ValueError(f"unsupported action: {action}")
            self._set_status(message)
            return {
                "accepted": True,
                "action": action,
                "hole": str(payload.get("hole", "--")).upper(),
                "message": message,
            }
        except Exception as error:
            self._set_status(f"ROS command rejected: {error}")
            return {
                "accepted": False,
                "action": action,
                "hole": str(payload.get("hole", "--")).upper(),
                "message": str(error),
            }

    def _run_selected(self) -> None:
        if not self._require_controller():
            return
        omni.timeline.get_timeline_interface().play()
        hole = HOLE_IDS[self._hole_combo.model.get_item_value_model().as_int]
        instruction = self._instruction.model.as_string or f"process {hole}"
        try:
            decision = self._controller.start_selected(hole, instruction)
            self._set_status(
                f"VLA decision: {decision.hole_id} | {decision.strategy} | {decision.source}"
            )
        except Exception as error:
            self._set_status(str(error))

    def _run_sequence(self) -> None:
        if not self._require_controller():
            return
        omni.timeline.get_timeline_interface().play()
        instruction = self._instruction.model.as_string or "process all pending DRPE holes in sequence"
        try:
            decision = self._controller.start_sequence(instruction)
            self._set_status(
                f"Batch started: {decision.hole_id} | {decision.strategy} | {decision.source}"
            )
        except Exception as error:
            self._set_status(str(error))

    def _pause_task(self) -> None:
        if self._require_controller():
            self._controller.pause()

    def _reset_batch(self) -> None:
        if self._require_controller():
            self._controller.reset()
            self._refresh_ui()

    def _toggle_centerlines(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        self._centerlines_visible = not self._centerlines_visible
        set_centerlines_visible(stage, self._centerlines_visible)
        self._set_status(
            f"Centerline visualization {'enabled' if self._centerlines_visible else 'hidden'}"
        )

    def _toggle_frames(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        self._frames_visible = not self._frames_visible
        set_frames_visible(stage, self._frames_visible)
        self._set_status(
            f"Coordinate frames {'enabled' if self._frames_visible else 'hidden'}"
        )

    def _toggle_colliders(self) -> None:
        self._colliders_visible = not self._colliders_visible
        value = 2 if self._colliders_visible else 0
        carb.settings.get_settings().set_int(
            "/persistent/physics/visualizationDisplayColliders",
            value,
        )
        self._set_status(
            f"Physics colliders {'visible' if self._colliders_visible else 'hidden'}"
        )

    def _save_scene(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._set_status("No scene is available to save")
            return
        output = PORTFOLIO_ROOT / "scenes" / "aero_drill_vla.usda"
        output.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output))
        self._set_status(f"Saved: {output}")

    def _play_simulation(self) -> None:
        omni.timeline.get_timeline_interface().play()
        self._set_status("Simulation Play")

    def _pause_simulation(self) -> None:
        omni.timeline.get_timeline_interface().pause()
        self._set_status("Simulation Pause")

    def _on_update(self, event) -> None:
        if self._controller is None:
            return
        dt = float(event.payload.get("dt", 1.0 / 60.0)) if event.payload else 1.0 / 60.0
        self._controller.update(dt)
        if self._ros_bridge is not None:
            self._ros_bridge.tick(dt, self._controller, self._status_text)
        self._refresh_ui()

    def _refresh_ui(self) -> None:
        controller = self._controller
        if controller is None:
            return
        cells = []
        for index, hole in enumerate(HOLE_IDS):
            marker = "PASS" if controller.completed[hole] else "WAIT"
            if hole == controller.active_hole:
                marker = "RUN"
            cells.append(f"{hole}:{marker}")
        self._hole_status_label.text = "  ".join(cells[:5]) + "\n" + "  ".join(cells[5:])
        self._active_label.text = f"Active hole: {controller.active_hole}"
        self._state_label.text = f"State: {controller.state}{' (PAUSED)' if controller.paused else ''}"
        self._strategy_label.text = f"Strategy: {controller.active_strategy}"
        self._force_label.text = f"Axial force: {controller.current_force_n:.1f} N"
        self._spindle_label.text = (
            f"Spindle: {controller.spindle_rpm} rpm | Feed: {controller.feed_mm_s:.1f} mm/s"
        )
        self._quality_label.text = (
            f"Last quality: {controller.last_quality:.1f}% | {controller.last_cycle_seconds:.1f}s"
            if controller.last_quality
            else "Last quality: --"
        )
        self._progress_label.text = f"Batch progress: {controller.completed_count} / 10"
        if self._ros_bridge is None:
            self._ros_label.text = "ROS 2: unavailable"
        elif self._ros_bridge.available:
            self._ros_label.text = (
                "ROS 2: CONNECTED | "
                f"RX commands {self._ros_bridge.command_count} | "
                f"TX messages {self._ros_bridge.publish_count}"
            )
        else:
            self._ros_label.text = f"ROS 2: unavailable | {self._ros_bridge.error}"
        self._robot_mode_label.text = f"Robot: {controller.robot_mode}"
        self._tcp_pose_label.text = controller.tcp_pose_text
        self._ik_label.text = f"IK / TCP tracking error: {controller.tcp_error_mm:.1f} mm"
        self._collision_label.text = controller.collision_summary
        active_frame = controller.active_hole if controller.active_hole != "--" else "none"
        self._frame_label.text = (
            f"Frames: W = UR Base B | TCP Z+ = drilling direction | "
            f"active hole frame = {active_frame} (Z+ outward)"
        )
        for label, row in zip(self._joint_labels, controller.joint_rows()):
            label.text = (
                f"{row['label']:<12} {row['position_deg']:+7.2f} deg | "
                f"{row['velocity_deg_s']:+6.2f} deg/s | "
                f"[{row['lower_deg']:+.0f}, {row['upper_deg']:+.0f}]"
            )

    def _set_status(self, message: str) -> None:
        self._status_text = message
        if self._status_label:
            self._status_label.text = message

    def _require_controller(self) -> bool:
        if self._controller is None:
            self._set_status("Create the scene first")
            return False
        return True
