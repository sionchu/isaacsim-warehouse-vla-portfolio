from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Callable

from pxr import Gf, UsdGeom

from .grid_planner import GridRoutePlanner, Point
from .scene_builder import ROBOT_PATH, SCENE_ROOT, SHELF_HEIGHTS, approach_position, slot_box_position
from .slot_policy import SLOT_IDS, SlotDecision, WarehouseVLAPolicy


class MissionController:
    """Runs the visual MVP mission while keeping slot choice behind a safe policy gate."""

    def __init__(
        self,
        stage,
        model_path: str | Path,
        event_log_path: str | Path,
        status_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.stage = stage
        self.policy = WarehouseVLAPolicy(model_path)
        self.route_planner = GridRoutePlanner()
        self.occupancy = {slot: False for slot in SLOT_IDS}
        self.event_log_path = Path(event_log_path)
        self.status_callback = status_callback
        self.state = "IDLE"
        self.current_position = Point(-6.0, 0.0)
        self.route: list[Point] = []
        self.route_index = 0
        self.active_decision: SlotDecision | None = None
        self.phase_elapsed = 0.0
        self.travel_speed = 1.1
        self._mission_started_at = 0.0

    def start(self, instruction: str, requested_slot: str | None = None) -> SlotDecision:
        if self.state != "IDLE":
            raise RuntimeError("A mission is already running.")
        decision = self.policy.choose(instruction, self.occupancy, requested_slot)
        goal_x, goal_y = approach_position(decision.slot_id)
        self.route = self.route_planner.plan(self.current_position, Point(goal_x, goal_y))
        self.route_index = 1 if len(self.route) > 1 else 0
        self.active_decision = decision
        self.state = "NAVIGATING"
        self.phase_elapsed = 0.0
        self._mission_started_at = time.time()
        self._show_active_cargo(True)
        self._draw_route(self.route)
        self._notify(
            f"Navigating to {decision.slot_id} | {decision.source} | confidence {decision.confidence:.2f}"
        )
        return decision

    def update(self, dt: float) -> None:
        if self.state == "IDLE" or self.active_decision is None:
            return
        self.phase_elapsed += dt
        if self.state == "NAVIGATING":
            if self._advance_route(dt):
                self.state = "LIFTING"
                self.phase_elapsed = 0.0
                self._notify(f"Arrived at {self.active_decision.slot_id} | Raising lift")
        elif self.state == "LIFTING":
            level = int(self.active_decision.slot_id[1:])
            target_height = SHELF_HEIGHTS[level] - SHELF_HEIGHTS[1]
            ratio = min(self.phase_elapsed / 1.5, 1.0)
            self._set_tray_height(target_height * ratio)
            if ratio >= 1.0:
                self._deposit_box()
                self.state = "LOWERING"
                self.phase_elapsed = 0.0
                self._notify(f"Stored at {self.active_decision.slot_id} | Lowering lift")
        elif self.state == "LOWERING":
            level = int(self.active_decision.slot_id[1:])
            start_height = SHELF_HEIGHTS[level] - SHELF_HEIGHTS[1]
            ratio = min(self.phase_elapsed / 1.2, 1.0)
            self._set_tray_height(start_height * (1.0 - ratio))
            if ratio >= 1.0:
                self.route = list(reversed(self.route))
                self.route_index = 1 if len(self.route) > 1 else 0
                self.state = "RETURNING"
                self.phase_elapsed = 0.0
        elif self.state == "RETURNING" and self._advance_route(dt):
            self._finish_mission()

    def release_slot(self, slot_id: str) -> None:
        if slot_id not in self.occupancy:
            raise ValueError(slot_id)
        self.occupancy[slot_id] = False
        path = f"{SCENE_ROOT}/StoredBoxes/{slot_id}"
        if self.stage.GetPrimAtPath(path).IsValid():
            self.stage.RemovePrim(path)
        self._notify(f"Cleared slot {slot_id}")

    def reset_storage(self) -> None:
        for slot in SLOT_IDS:
            self.occupancy[slot] = False
            path = f"{SCENE_ROOT}/StoredBoxes/{slot}"
            if self.stage.GetPrimAtPath(path).IsValid():
                self.stage.RemovePrim(path)
        self._notify("Reset all slots")

    def complete_external(self, decision: SlotDecision, elapsed: float = 0.0) -> None:
        """Reflect a completed Nav2 mission in the scene and mission log."""
        if self.occupancy.get(decision.slot_id, False):
            return
        self.active_decision = decision
        self._deposit_box()
        self._log_event(decision, elapsed)
        self.active_decision = None
        self._show_active_cargo(True)
        self._notify(f"Nav2 mission complete | {decision.slot_id}")

    def occupancy_text(self) -> str:
        rows = []
        for rack in ("B", "C"):
            cells = [
                f"{rack}{level}:{'FULL' if self.occupancy[f'{rack}{level}'] else 'EMPTY'}"
                for level in range(1, 6)
            ]
            rows.append("  ".join(cells))
        return "\n".join(rows)

    def _advance_route(self, dt: float) -> bool:
        if self.route_index >= len(self.route):
            return True
        target = self.route[self.route_index]
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        distance = math.hypot(dx, dy)
        step = self.travel_speed * dt
        if distance <= step or distance < 1e-4:
            self.current_position = target
            self.route_index += 1
        else:
            self.current_position = Point(
                self.current_position.x + dx / distance * step,
                self.current_position.y + dy / distance * step,
            )
        self._set_robot_position(self.current_position)
        return self.route_index >= len(self.route)

    def _set_robot_position(self, point: Point) -> None:
        prim = self.stage.GetPrimAtPath(ROBOT_PATH)
        if prim.IsValid():
            position = prim.GetAttribute("xformOp:translate:portfolio")
            if position.IsValid():
                position.Set(Gf.Vec3d(point.x, point.y, 0.0))
            else:
                UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(point.x, point.y, 0.0))

    def _set_tray_height(self, lift_offset: float) -> None:
        for path, z in (
            (f"{ROBOT_PATH}/PortfolioLift/Tray", 0.65 + lift_offset),
            (f"{ROBOT_PATH}/PortfolioLift/ActiveCargo", 0.84 + lift_offset),
        ):
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(0.0, 0.0, z))

    def _deposit_box(self) -> None:
        slot_id = self.active_decision.slot_id
        self.occupancy[slot_id] = True
        root = f"{SCENE_ROOT}/StoredBoxes"
        self.stage.DefinePrim(root, "Xform")
        cube = UsdGeom.Cube.Define(self.stage, f"{root}/{slot_id}")
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.76, 0.52, 0.20)])
        transform = UsdGeom.XformCommonAPI(cube.GetPrim())
        transform.SetTranslate(Gf.Vec3d(*slot_box_position(slot_id)))
        transform.SetScale(Gf.Vec3f(0.26, 0.30, 0.16))
        self._show_active_cargo(False)

    def _show_active_cargo(self, visible: bool) -> None:
        prim = self.stage.GetPrimAtPath(f"{ROBOT_PATH}/PortfolioLift/ActiveCargo")
        if prim.IsValid():
            imageable = UsdGeom.Imageable(prim)
            imageable.MakeVisible() if visible else imageable.MakeInvisible()

    def _finish_mission(self) -> None:
        elapsed = time.time() - self._mission_started_at
        decision = self.active_decision
        self._log_event(decision, elapsed)
        self.active_decision = None
        self.state = "IDLE"
        self.phase_elapsed = 0.0
        self.route = []
        self.route_index = 0
        self._set_tray_height(0.0)
        self._show_active_cargo(True)
        self._clear_route()
        self._notify(f"Mission complete | {decision.slot_id} | {elapsed:.1f}s")

    def _draw_route(self, route: list[Point]) -> None:
        self._clear_route()
        route_root = f"{SCENE_ROOT}/PlannedRoute"
        self.stage.DefinePrim(route_root, "Xform")
        for index, point in enumerate(route):
            marker = UsdGeom.Sphere.Define(self.stage, f"{route_root}/P{index:02d}")
            marker.CreateRadiusAttr(0.07)
            marker.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.95, 0.25)])
            UsdGeom.XformCommonAPI(marker.GetPrim()).SetTranslate(Gf.Vec3d(point.x, point.y, 0.08))

    def _clear_route(self) -> None:
        path = f"{SCENE_ROOT}/PlannedRoute"
        if self.stage.GetPrimAtPath(path).IsValid():
            self.stage.RemovePrim(path)

    def _log_event(self, decision: SlotDecision, elapsed: float) -> None:
        self.event_log_path.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "timestamp": time.time(),
            "instruction": decision.instruction,
            "slot": decision.slot_id,
            "policy_source": decision.source,
            "confidence": decision.confidence,
            "elapsed_seconds": elapsed,
            "occupancy": self.occupancy,
        }
        with self.event_log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _notify(self, message: str) -> None:
        if self.status_callback:
            self.status_callback(message)
