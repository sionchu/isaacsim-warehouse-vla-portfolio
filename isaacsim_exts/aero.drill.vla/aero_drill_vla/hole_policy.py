from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

HOLE_IDS = tuple(f"H{index:02d}" for index in range(1, 11))


@dataclass(frozen=True)
class HoleObservation:
    complete: bool
    position_error_mm: float
    normal_error_deg: float
    material_stack: str


@dataclass(frozen=True)
class DrillDecision:
    hole_id: str
    strategy: str
    source: str
    confidence: float
    instruction: str


class AeroDrillVLAPolicy:
    """Selects a hole with VLA-lite while a deterministic gate selects docking strategy."""

    def __init__(self, model_path: str | Path | None = None) -> None:
        self.model_path = Path(model_path) if model_path else None
        self._checkpoint = None
        self._model = None
        self._torch = None
        if self.model_path and self.model_path.is_file():
            self._load_model()

    @property
    def mode(self) -> str:
        return "trained aero VLA-lite" if self._model is not None else "expert bootstrap"

    def choose(
        self,
        instruction: str,
        observations: Mapping[str, HoleObservation],
        requested_hole: str | None = None,
    ) -> DrillDecision:
        pending = [hole for hole in HOLE_IDS if not observations[hole].complete]
        if not pending:
            raise RuntimeError("All ten DRPE holes are complete.")
        if requested_hole:
            if requested_hole not in HOLE_IDS:
                raise ValueError(f"Unknown hole: {requested_hole}")
            if observations[requested_hole].complete:
                raise RuntimeError(f"{requested_hole} is already complete.")

        if self._model is not None:
            predicted, confidence = self._predict(instruction, observations, pending)
            if requested_hole and predicted != requested_hole:
                selected = requested_hole
                source = "manual safety override"
            else:
                selected = predicted
                source = "trained aero VLA-lite"
        else:
            selected = requested_hole or pending[0]
            confidence = 1.0
            source = "manual expert" if requested_hole else "sequential expert"

        observation = observations[selected]
        if observation.position_error_mm > 0.80 or observation.normal_error_deg > 0.85:
            strategy = "SPIRAL SEARCH"
        elif observation.position_error_mm > 0.30 or observation.normal_error_deg > 0.35:
            strategy = "VISION REFINE"
        else:
            strategy = "DIRECT DOCK"
        return DrillDecision(selected, strategy, source, confidence, instruction)

    def _load_model(self) -> None:
        try:
            import torch

            from .vla_model import TinyAeroDrillVLA

            checkpoint = torch.load(self.model_path, map_location="cpu", weights_only=False)
            model = TinyAeroDrillVLA(
                vocab_size=len(checkpoint["char_to_index"]) + 1,
                embedding_dim=checkpoint["embedding_dim"],
                hidden_dim=checkpoint["hidden_dim"],
            )
            model.load_state_dict(checkpoint["state_dict"])
            model.eval()
            self._checkpoint = checkpoint
            self._model = model
            self._torch = torch
        except Exception:
            self._checkpoint = None
            self._model = None
            self._torch = None

    def _predict(
        self,
        instruction: str,
        observations: Mapping[str, HoleObservation],
        pending: Sequence[str],
    ) -> tuple[str, float]:
        torch = self._torch
        checkpoint = self._checkpoint
        char_to_index = checkpoint["char_to_index"]
        max_length = checkpoint["max_length"]
        encoded = [char_to_index.get(character, 0) for character in instruction[:max_length]]
        encoded += [0] * (max_length - len(encoded))
        text = torch.tensor([encoded], dtype=torch.long)
        vision = torch.zeros((1, 3, 2, 5), dtype=torch.float32)
        for index, hole in enumerate(HOLE_IDS):
            row, column = divmod(index, 5)
            observation = observations[hole]
            vision[0, 0, row, column] = 1.0 if observation.complete else 0.0
            vision[0, 1, row, column] = min(observation.position_error_mm / 2.0, 1.0)
            vision[0, 2, row, column] = min(observation.normal_error_deg / 2.0, 1.0)

        with torch.no_grad():
            logits = self._model(text, vision)[0]
            for index, hole in enumerate(HOLE_IDS):
                if hole not in pending:
                    logits[index] = -1e9
            probabilities = torch.softmax(logits, dim=0)
            index = int(torch.argmax(probabilities).item())
        return HOLE_IDS[index], float(probabilities[index].item())
