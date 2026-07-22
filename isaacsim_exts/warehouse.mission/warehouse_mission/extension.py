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
                ui.Label("A \u2192 B/C 5\ub2e8 \uac04\ubc18 \uc790\ub3d9 \ubcf4\uad00", style={"font_size": 20})
                ui.Label(
                    "VLA-lite \ubaa9\ud45c \uc120\ud0dd + A*/Nav2 \uc548\uc804 \uc8fc\ud589",
                    style={"color": 0xFF9AD7FF},
                )
                ui.Separator()

                with ui.HStack(height=30):
                    ui.Label("\uc2e4\ud589 \ubaa8\ub4dc", width=90)
                    self._engine_combo = ui.ComboBox(0, "Visual A*", "ROS2 Nav2")
                with ui.HStack(height=30):
                    ui.Label("\ubaa9\uc801\uc9c0", width=90)
                    self._rack_combo = ui.ComboBox(0, "B", "C")
                with ui.HStack(height=30):
                    ui.Label("\uce35", width=90)
                    self._level_combo = ui.ComboBox(0, "1\uce35", "2\uce35", "3\uce35", "4\uce35", "5\uce35")
                with ui.HStack(height=30):
                    ui.Label("\uc5b8\uc5b4 \uba85\ub839", width=90)
                    self._instruction = ui.StringField()
                    self._instruction.model.set_value("\uc120\ud0dd\ud55c \uac04\ubc18\uc73c\ub85c \ubc15\uc2a4\ub97c \uc6b4\ubc18\ud574")

                with ui.HStack(height=34, spacing=6):
                    ui.Button("\uc218\ub3d9 \ubaa9\uc801\uc9c0 \uc2e4\ud589", clicked_fn=self._manual_dispatch)
                    ui.Button("\uc790\ub3d9 \ube48 \uc2ac\ub86f", clicked_fn=self._auto_dispatch)

                ui.Separator()
                ui.Label("\uc2ac\ub86f \uc0c1\ud0dc")
                self._occupancy_label = ui.Label(
                    "B1:EMPTY B2:EMPTY B3:EMPTY B4:EMPTY B5:EMPTY\n"
                    "C1:EMPTY C2:EMPTY C3:EMPTY C4:EMPTY C5:EMPTY"
                )

                with ui.HStack(height=34, spacing=6):
                    ui.Button("\uc120\ud0dd \uc2ac\ub86f \ube44\uc6b0\uae30", clicked_fn=self._release_selected)
                    ui.Button("\uc804\uccb4 \ucd08\uae30\ud654", clicked_fn=self._reset_storage)

                ui.Separator()
                with ui.HStack(height=34, spacing=6):
                    ui.Button("\uc7a5\uba74 \uc7ac\uc0dd\uc131", clicked_fn=self._create_scene)
                    ui.Button("USD \uc800\uc7a5", clicked_fn=self._save_scene)

                with ui.HStack(height=34, spacing=6):
                    ui.Button("Simulation Play", clicked_fn=self._play_simulation)
                    ui.Button("Simulation Pause", clicked_fn=self._pause_simulation)

                self._policy_label = ui.Label("\uc815\ucc45: \ucd08\uae30\ud654 \uc911")
                self._status_label = ui.Label("\ub300\uae30 \uc911", word_wrap=True)
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
        self._policy_label.text = f"\uc815\ucc45: {self._controller.policy.mode}"
        self._refresh_occupancy()
        self._set_status("\uc7a5\uba74 \uc0dd\uc131 \uc644\ub8cc | A \uad6c\uc5ed\uc5d0\uc11c \uc784\ubb34\ub97c \uc120\ud0dd\ud558\uc138\uc694")

    def _save_scene(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._set_status("\uc800\uc7a5\ud560 \uc7a5\uba74\uc774 \uc5c6\uc2b5\ub2c8\ub2e4")
            return
        output = PORTFOLIO_ROOT / "scenes" / "warehouse_mission.usda"
        output.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output))
        self._set_status(f"\uc800\uc7a5 \uc644\ub8cc: {output}")

    def _play_simulation(self) -> None:
        omni.timeline.get_timeline_interface().play()
        self._set_status("Simulation Play | ROS 2 \uc13c\uc11c \ucd9c\ub825 \ud65c\uc131\ud654")

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
            f"{rack} \uad6c\uc5ed {level}\uce35\uc5d0 \ubc15\uc2a4\ub97c \ubcf4\uad00\ud574"
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
        instruction = self._instruction.model.as_string or "\ube44\uc5b4 \uc788\ub294 \uac04\ubc18\uc5d0 \uc790\ub3d9 \ubcf4\uad00\ud574"
        try:
            if self._uses_nav2():
                decision = self._controller.policy.choose(instruction, self._controller.occupancy)
                self._dispatch_ros(decision)
            else:
                decision = self._controller.start(instruction)
                self._set_status(f"VLA \uacb0\uc815: {decision.slot_id} | {decision.source}")
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
            self._set_status("\uba3c\uc800 \uc7a5\uba74\uc744 \uc0dd\uc131\ud558\uc138\uc694")
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
        self._set_status(f"Nav2 \uc694\uccad \uc804\uc1a1 | {decision.slot_id} | {decision.source}")

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
