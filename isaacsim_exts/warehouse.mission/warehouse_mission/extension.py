from __future__ import annotations

from pathlib import Path
import time

import omni.ext
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd

from .mission_controller import MissionController
from .ros_bridge import RosMissionBridge
from .scene_builder import build_scene
from .slot_policy import SlotDecision

PORTFOLIO_ROOT = Path(__file__).resolve().parents[3]


class WarehouseMissionExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        self._ext_id = ext_id
        self._window = ui.Window("Warehouse Mission Control", width=440, height=590)
        self._controller = None
        self._pending_ros_decision: SlotDecision | None = None
        self._pending_ros_started_at = 0.0
        self._ros_bridge = RosMissionBridge(self._on_ros_status)
        self._update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update, name="warehouse_mission_update")
        )
        self._build_ui()
        self._create_scene()

    def on_shutdown(self) -> None:
        if self._ros_bridge:
            self._ros_bridge.close()
        self._ros_bridge = None
        self._update_subscription = None
        self._controller = None
        self._window = None

    def _build_ui(self) -> None:
        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("A -> B/C Warehouse Storage", style={"font_size": 20})
                ui.Label(
                    "VLA-lite target selection + A*/Nav2 navigation",
                    style={"color": 0xFF9AD7FF},
                )
                ui.Separator()

                with ui.HStack(height=30):
                    ui.Label("Run mode", width=90)
                    self._engine_combo = ui.ComboBox(0, "Visual A*", "ROS2 Nav2")
                with ui.HStack(height=30):
                    ui.Label("Rack", width=90)
                    self._rack_combo = ui.ComboBox(0, "B", "C")
                with ui.HStack(height=30):
                    ui.Label("Level", width=90)
                    self._level_combo = ui.ComboBox(0, "Level 1", "Level 2", "Level 3", "Level 4", "Level 5")
                with ui.HStack(height=30):
                    ui.Label("Instruction", width=90)
                    self._instruction = ui.StringField()
                    self._instruction.model.set_value("Move the box to the selected rack")

                with ui.HStack(height=34, spacing=6):
                    ui.Button("Run Selected Target", clicked_fn=self._manual_dispatch)
                    ui.Button("Auto Free Slot", clicked_fn=self._auto_dispatch)

                ui.Separator()
                ui.Label("Slot Occupancy")
                self._occupancy_label = ui.Label(
                    "B1:EMPTY B2:EMPTY B3:EMPTY B4:EMPTY B5:EMPTY\n"
                    "C1:EMPTY C2:EMPTY C3:EMPTY C4:EMPTY C5:EMPTY"
                )

                with ui.HStack(height=34, spacing=6):
                    ui.Button("Clear Selected Slot", clicked_fn=self._release_selected)
                    ui.Button("Reset All Slots", clicked_fn=self._reset_storage)

                ui.Separator()
                with ui.HStack(height=34, spacing=6):
                    ui.Button("Rebuild Scene", clicked_fn=self._create_scene)
                    ui.Button("Save USD", clicked_fn=self._save_scene)

                with ui.HStack(height=34, spacing=6):
                    ui.Button("Simulation Play", clicked_fn=self._play_simulation)
                    ui.Button("Simulation Pause", clicked_fn=self._pause_simulation)

                self._policy_label = ui.Label("Policy: initializing")
                self._status_label = ui.Label("Ready", word_wrap=True)
                ui.Spacer(height=8)
                ui.Label("Camera: /front_stereo_camera/left/image_raw", style={"color": 0xFFAAAAAA})
                ui.Label("SLAM: /front_2d_lidar/scan", style={"color": 0xFFAAAAAA})
                ui.Label("Monitor: scripts/start_monitor.ps1", style={"color": 0xFFAAAAAA})

    def _create_scene(self) -> None:
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        output = PORTFOLIO_ROOT / "scenes" / "warehouse_mission.usda"
        build_scene(stage, output)
        self._controller = MissionController(
            stage,
            PORTFOLIO_ROOT / "models" / "warehouse_vla.pt",
            PORTFOLIO_ROOT / "recordings" / "mission_events.jsonl",
            self._set_status,
        )
        self._policy_label.text = f"Policy: {self._controller.policy.mode}"
        self._refresh_occupancy()
        self._set_status("Scene ready | Select a mission at loading zone A")

    def _save_scene(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._set_status("No scene is available to save")
            return
        output = PORTFOLIO_ROOT / "scenes" / "warehouse_mission.usda"
        output.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output))
        self._set_status(f"Saved: {output}")

    def _play_simulation(self) -> None:
        omni.timeline.get_timeline_interface().play()
        self._set_status("Simulation Play | ROS 2 sensor output enabled")

    def _pause_simulation(self) -> None:
        omni.timeline.get_timeline_interface().pause()
        self._set_status("Simulation Pause")

    def _manual_dispatch(self) -> None:
        if not self._require_controller():
            return
        rack = ("B", "C")[self._rack_combo.model.get_item_value_model().as_int]
        level = self._level_combo.model.get_item_value_model().as_int + 1
        slot = f"{rack}{level}"
        instruction = self._instruction.model.as_string or (
            f"Store the box at rack {rack}, level {level}"
        )
        try:
            if self._uses_nav2():
                decision = self._controller.policy.choose(
                    instruction, self._controller.occupancy, requested_slot=slot
                )
                self._dispatch_ros(decision)
            else:
                self._controller.start(instruction, requested_slot=slot)
        except Exception as error:
            self._set_status(str(error))

    def _auto_dispatch(self) -> None:
        if not self._require_controller():
            return
        instruction = self._instruction.model.as_string or "Store the box in an available slot"
        try:
            if self._uses_nav2():
                decision = self._controller.policy.choose(instruction, self._controller.occupancy)
                self._dispatch_ros(decision)
            else:
                decision = self._controller.start(instruction)
                self._set_status(f"VLA decision: {decision.slot_id} | {decision.source}")
        except Exception as error:
            self._set_status(str(error))

    def _release_selected(self) -> None:
        if not self._require_controller():
            return
        rack = ("B", "C")[self._rack_combo.model.get_item_value_model().as_int]
        level = self._level_combo.model.get_item_value_model().as_int + 1
        self._controller.release_slot(f"{rack}{level}")
        self._refresh_occupancy()

    def _reset_storage(self) -> None:
        if self._require_controller():
            self._controller.reset_storage()
            self._refresh_occupancy()

    def _require_controller(self) -> bool:
        if self._controller is None:
            self._set_status("Create the scene first")
            return False
        return True

    def _on_update(self, event) -> None:
        if self._ros_bridge:
            self._ros_bridge.spin_once()
        if self._controller is None:
            return
        dt = float(event.payload.get("dt", 1.0 / 60.0)) if event.payload else 1.0 / 60.0
        previous_state = self._controller.state
        self._controller.update(min(dt, 0.1))
        if previous_state != self._controller.state or self._controller.state == "IDLE":
            self._refresh_occupancy()

    def _refresh_occupancy(self) -> None:
        if self._controller:
            self._occupancy_label.text = self._controller.occupancy_text()

    def _set_status(self, message: str) -> None:
        if self._status_label:
            self._status_label.text = message

    def _uses_nav2(self) -> bool:
        return self._engine_combo.model.get_item_value_model().as_int == 1

    def _dispatch_ros(self, decision: SlotDecision) -> None:
        if self._pending_ros_decision is not None:
            raise RuntimeError("A Nav2 mission is already running.")
        self._ros_bridge.publish(
            decision.slot_id,
            decision.instruction,
            decision.confidence,
            decision.source,
        )
        self._pending_ros_decision = decision
        self._pending_ros_started_at = time.time()
        self._set_status(f"Nav2 request sent | {decision.slot_id} | {decision.source}")

    def _on_ros_status(self, payload: dict) -> None:
        status = str(payload.get("status", "UNKNOWN"))
        slot = str(payload.get("slot", ""))
        detail = str(payload.get("detail", ""))
        self._set_status(f"Nav2 {status} | {slot} | {detail}")
        if self._pending_ros_decision is None or slot != self._pending_ros_decision.slot_id:
            return
        if status == "SUCCEEDED":
            elapsed = time.time() - self._pending_ros_started_at
            self._controller.complete_external(self._pending_ros_decision, elapsed)
            self._pending_ros_decision = None
            self._refresh_occupancy()
        elif status in {"FAILED", "CANCELED", "REJECTED"}:
            self._pending_ros_decision = None
