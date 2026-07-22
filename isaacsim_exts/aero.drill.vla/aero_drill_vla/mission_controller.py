from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Callable

from pxr import Gf

from .hole_policy import AeroDrillVLAPolicy, DrillDecision, HOLE_IDS, HoleObservation
from .scene_builder import hole_frames, set_hole_status, update_cobot_visual

HOME_TCP = (0.28, 0.0, 1.48)
HOME_AXIS = (-1.0, 0.0, 0.0)


class AeroDrillMissionController:
    """Deterministic drilling-cycle surrogate with VLA-level target selection."""

    def __init__(
        self,
        stage,
        model_path: str | Path,
        event_log_path: str | Path,
        status_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.stage = stage
        self.policy = AeroDrillVLAPolicy(model_path)
        self.event_log_path = Path(event_log_path)
        self.status_callback = status_callback
        self.frames = hole_frames()
        self.completed = {hole: False for hole in HOLE_IDS}
        self.cycle_order: list[str] = []
        self.state = "IDLE"
        self.paused = False
        self.sequence_enabled = False
        self.sequence_instruction = "process the next pending DRPE hole"
        self.active_decision: DrillDecision | None = None
        self.current_tcp = Gf.Vec3d(*HOME_TCP)
        self.current_axis = Gf.Vec3d(*HOME_AXIS)
        self.phase_start = Gf.Vec3d(*HOME_TCP)
        self.phase_target = Gf.Vec3d(*HOME_TCP)
        self.phase_elapsed = 0.0
        self.phase_duration = 1.0
        self.cycle_speed = 1.25
        self.current_force_n = 0.0
        self.spindle_rpm = 0
        self.feed_mm_s = 0.0
        self.last_quality = 0.0
        self.last_cycle_seconds = 0.0
        self._cycle_started_at = 0.0
        update_cobot_visual(self.stage, self.current_tcp, self.current_axis)

    @property
    def completed_count(self) -> int:
        return sum(self.completed.values())

    @property
    def active_hole(self) -> str:
        return self.active_decision.hole_id if self.active_decision else "--"

    @property
    def active_strategy(self) -> str:
        return self.active_decision.strategy if self.active_decision else "--"

    def observations(self) -> dict[str, HoleObservation]:
        return {
            hole_id: HoleObservation(
                complete=self.completed[hole_id],
                position_error_mm=frame.position_error_mm,
                normal_error_deg=frame.normal_error_deg,
                material_stack=frame.material_stack,
            )
            for hole_id, frame in self.frames.items()
        }

    def start_selected(self, hole_id: str, instruction: str) -> DrillDecision:
        self._require_idle()
        self.sequence_enabled = False
        decision = self.policy.choose(instruction, self.observations(), requested_hole=hole_id)
        self._begin_cycle(decision)
        return decision

    def start_sequence(self, instruction: str) -> DrillDecision:
        self._require_idle()
        self.sequence_enabled = True
        self.sequence_instruction = instruction or "process the next pending DRPE hole"
        decision = self._choose_next_in_sequence()
        self._begin_cycle(decision)
        return decision

    def pause(self) -> None:
        self.paused = not self.paused
        self._notify("Task paused" if self.paused else f"Task resumed | {self.state}")

    def reset(self) -> None:
        self.completed = {hole: False for hole in HOLE_IDS}
        self.cycle_order = []
        self.state = "IDLE"
        self.paused = False
        self.sequence_enabled = False
        self.active_decision = None
        self.current_tcp = Gf.Vec3d(*HOME_TCP)
        self.current_axis = Gf.Vec3d(*HOME_AXIS)
        self.current_force_n = 0.0
        self.spindle_rpm = 0
        self.feed_mm_s = 0.0
        self.last_quality = 0.0
        self.last_cycle_seconds = 0.0
        for hole_id in HOLE_IDS:
            set_hole_status(self.stage, hole_id, "PENDING")
        update_cobot_visual(self.stage, self.current_tcp, self.current_axis)
        self._notify("Batch reset | Ten DRPE holes pending")

    def update(self, dt: float) -> None:
        if self.paused or self.state == "IDLE" or self.active_decision is None:
            return
        scaled_dt = min(max(dt, 0.0), 0.1) * self.cycle_speed
        self.phase_elapsed += scaled_dt
        ratio = min(self.phase_elapsed / max(self.phase_duration, 1e-6), 1.0)
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        self.current_tcp = self.phase_start * (1.0 - smooth) + self.phase_target * smooth

        if self.state == "SEARCH":
            frame = self.frames[self.active_decision.hole_id]
            amplitude = 0.004 * (1.0 - ratio)
            angle = ratio * math.tau * 2.0
            self.current_tcp += Gf.Vec3d(0.0, math.cos(angle) * amplitude, math.sin(angle) * amplitude)

        self._update_process_telemetry(ratio)
        active = self.state in {"DOCK", "CLAMP", "DRILL", "VERIFY"}
        update_cobot_visual(self.stage, self.current_tcp, self.current_axis, active=active)
        if ratio >= 1.0:
            self._advance_state()

    def _begin_cycle(self, decision: DrillDecision) -> None:
        self.active_decision = decision
        self._cycle_started_at = time.time()
        frame = self.frames[decision.hole_id]
        self.current_axis = Gf.Vec3d(*frame.outward)
        set_hole_status(self.stage, decision.hole_id, "ACTIVE")
        approach = Gf.Vec3d(*frame.center) + self.current_axis * 0.42
        self._start_phase("APPROACH", approach, 0.95)
        self._notify(
            f"{decision.hole_id} approach | {decision.strategy} | confidence {decision.confidence:.2f}"
        )

    def _advance_state(self) -> None:
        decision = self.active_decision
        frame = self.frames[decision.hole_id]
        center = Gf.Vec3d(*frame.center)
        outward = Gf.Vec3d(*frame.outward)
        if self.state == "APPROACH":
            self._start_phase("ALIGN", center + outward * 0.17, 0.62)
            self._notify(
                f"{decision.hole_id} normal alignment | residual {frame.position_error_mm:.2f} mm / {frame.normal_error_deg:.2f} deg"
            )
        elif self.state == "ALIGN" and decision.strategy != "DIRECT DOCK":
            duration = 0.85 if decision.strategy == "VISION REFINE" else 1.25
            self._start_phase("SEARCH", center + outward * 0.15, duration)
            self._notify(f"{decision.hole_id} {decision.strategy.lower()} | centerline reacquisition")
        elif self.state in {"ALIGN", "SEARCH"}:
            self._start_phase("DOCK", center + outward * 0.020, 0.56)
            self._notify(f"{decision.hole_id} DRPE bushing docking")
        elif self.state == "DOCK":
            self._start_phase("CLAMP", center + outward * 0.012, 0.38)
            self._notify(f"{decision.hole_id} concentric collet clamp")
        elif self.state == "CLAMP":
            self._start_phase("DRILL", center - outward * 0.018, 0.92)
            self._notify(f"{decision.hole_id} drilling surrogate | {frame.material_stack}")
        elif self.state == "DRILL":
            self._start_phase("VERIFY", center - outward * 0.018, 0.30)
            self._notify(f"{decision.hole_id} depth / force verification")
        elif self.state == "VERIFY":
            self._finish_quality_record()
            self._start_phase("RETRACT", center + outward * 0.42, 0.76)
            self._notify(f"{decision.hole_id} PASS | quality {self.last_quality:.1f}%")
        elif self.state == "RETRACT":
            completed_hole = decision.hole_id
            self.completed[completed_hole] = True
            self.cycle_order.append(completed_hole)
            set_hole_status(self.stage, completed_hole, "COMPLETE")
            self.active_decision = None
            self.current_force_n = 0.0
            self.spindle_rpm = 0
            self.feed_mm_s = 0.0
            if self.sequence_enabled and self.completed_count < len(HOLE_IDS):
                next_decision = self._choose_next_in_sequence()
                self._begin_cycle(next_decision)
            else:
                self.state = "IDLE"
                self.sequence_enabled = False
                self._notify(
                    f"Batch complete | {self.completed_count}/10 holes | last cycle {self.last_cycle_seconds:.1f}s"
                )

    def _start_phase(self, state: str, target: Gf.Vec3d, duration: float) -> None:
        self.state = state
        self.phase_start = Gf.Vec3d(self.current_tcp)
        self.phase_target = Gf.Vec3d(target)
        self.phase_elapsed = 0.0
        self.phase_duration = duration

    def _choose_next_in_sequence(self) -> DrillDecision:
        observations = self.observations()
        expected = next(hole for hole in HOLE_IDS if not observations[hole].complete)
        decision = self.policy.choose(
            self.sequence_instruction,
            observations,
            requested_hole=expected,
        )
        if decision.source == "manual safety override":
            decision = DrillDecision(
                hole_id=decision.hole_id,
                strategy=decision.strategy,
                source="H01-H10 sequence safety gate",
                confidence=decision.confidence,
                instruction=decision.instruction,
            )
        return decision

    def _update_process_telemetry(self, ratio: float) -> None:
        if self.state in {"APPROACH", "ALIGN", "SEARCH", "RETRACT"}:
            self.current_force_n = 0.0 if self.state != "SEARCH" else 4.0 + 3.0 * abs(math.sin(ratio * math.tau))
            self.spindle_rpm = 0
            self.feed_mm_s = 0.0
        elif self.state == "DOCK":
            self.current_force_n = 6.0 + 8.0 * ratio
            self.spindle_rpm = 0
            self.feed_mm_s = 3.0
        elif self.state == "CLAMP":
            self.current_force_n = 14.0 + 31.0 * ratio
            self.spindle_rpm = 0
            self.feed_mm_s = 0.0
        elif self.state == "DRILL":
            self.current_force_n = 48.0 + 16.0 * math.sin(ratio * math.pi)
            self.spindle_rpm = 4800
            self.feed_mm_s = 5.5
        elif self.state == "VERIFY":
            self.current_force_n = 12.0
            self.spindle_rpm = 0
            self.feed_mm_s = 0.0

    def _finish_quality_record(self) -> None:
        decision = self.active_decision
        frame = self.frames[decision.hole_id]
        strategy_bonus = {"DIRECT DOCK": 0.0, "VISION REFINE": 0.65, "SPIRAL SEARCH": 1.15}[decision.strategy]
        self.last_quality = min(
            99.9,
            98.4 - frame.position_error_mm * 0.35 - frame.normal_error_deg * 0.25 + strategy_bonus,
        )
        self.last_cycle_seconds = time.time() - self._cycle_started_at
        record = {
            "timestamp": time.time(),
            "hole": decision.hole_id,
            "instruction": decision.instruction,
            "policy_source": decision.source,
            "confidence": decision.confidence,
            "strategy": decision.strategy,
            "position_error_mm": frame.position_error_mm,
            "normal_error_deg": frame.normal_error_deg,
            "material_stack": frame.material_stack,
            "peak_force_n": round(60.0 + frame.position_error_mm * 4.0, 2),
            "spindle_rpm": 4800,
            "feed_mm_s": 5.5,
            "quality_score": round(self.last_quality, 2),
            "elapsed_seconds": round(self.last_cycle_seconds, 3),
        }
        self.event_log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.event_log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _require_idle(self) -> None:
        if self.state != "IDLE":
            raise RuntimeError("A drilling cycle is already running.")
        if self.completed_count >= len(HOLE_IDS):
            raise RuntimeError("Batch is complete. Reset the batch before starting again.")

    def _notify(self, message: str) -> None:
        if self.status_callback:
            self.status_callback(message)
