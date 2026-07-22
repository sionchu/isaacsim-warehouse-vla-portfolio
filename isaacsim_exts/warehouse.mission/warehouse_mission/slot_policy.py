from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

SLOT_IDS = tuple(f"{rack}{level}" for rack in ("B", "C") for level in range(1, 6))


@dataclass(frozen=True)
class SlotDecision:
    slot_id: str
    source: str
    confidence: float
    instruction: str


class WarehouseVLAPolicy:
    """Small task-specific vision-language-action policy with a safe fallback."""

    def __init__(self, model_path: str | Path | None = None) -> None:
        self.model_path = Path(model_path) if model_path else None
        self._checkpoint = None
        self._model = None
        self._torch = None
        if self.model_path and self.model_path.is_file():
            self._load_model()

    @property
    def mode(self) -> str:
        return "trained VLA-lite" if self._model is not None else "expert bootstrap"

    def choose(
        self,
        instruction: str,
        occupancy: Mapping[str, bool],
        requested_slot: str | None = None,
    ) -> SlotDecision:
        free_slots = [slot for slot in SLOT_IDS if not occupancy.get(slot, False)]
        if not free_slots:
            raise RuntimeError("All B/C kanban slots are occupied.")

        if requested_slot:
            if requested_slot not in SLOT_IDS:
                raise ValueError(f"Unknown slot: {requested_slot}")
            if occupancy.get(requested_slot, False):
                raise RuntimeError(f"Requested slot {requested_slot} is already occupied.")

        if self._model is not None:
            predicted, confidence = self._predict(instruction, occupancy, free_slots)
            if requested_slot and predicted != requested_slot:
                # Manual UI selection is a hard safety constraint.
                return SlotDecision(requested_slot, "manual safety override", confidence, instruction)
            return SlotDecision(predicted, "trained VLA-lite", confidence, instruction)

        if requested_slot:
            return SlotDecision(requested_slot, "manual expert", 1.0, instruction)

        # Deterministic expert demonstrations prefer lower levels and then shorter B-side travel.
        selected = sorted(free_slots, key=lambda slot: (int(slot[1:]), slot[0]))[0]
        return SlotDecision(selected, "automatic expert", 1.0, instruction)

    def _load_model(self) -> None:
        try:
            import torch

            from .vla_model import TinyWarehouseVLA

            checkpoint = torch.load(self.model_path, map_location="cpu", weights_only=False)
            model = TinyWarehouseVLA(
                vocab_size=len(checkpoint["char_to_index"]) + 1,
                embedding_dim=checkpoint["embedding_dim"],
                hidden_dim=checkpoint["hidden_dim"],
            )
            model.load_state_dict(checkpoint["state_dict"])
            model.eval()
            self._torch = torch
            self._checkpoint = checkpoint
            self._model = model
        except Exception:
            self._checkpoint = None
            self._model = None
            self._torch = None

    def _predict(
        self,
        instruction: str,
        occupancy: Mapping[str, bool],
        free_slots: Sequence[str],
    ) -> tuple[str, float]:
        torch = self._torch
        checkpoint = self._checkpoint
        char_to_index = checkpoint["char_to_index"]
        max_length = checkpoint["max_length"]
        encoded = [char_to_index.get(character, 0) for character in instruction[:max_length]]
        encoded += [0] * (max_length - len(encoded))
        text = torch.tensor([encoded], dtype=torch.long)
        vision = torch.tensor(
            [[[[1.0 if occupancy.get(slot, False) else 0.0 for slot in SLOT_IDS[:5]],
               [1.0 if occupancy.get(slot, False) else 0.0 for slot in SLOT_IDS[5:]]]]],
            dtype=torch.float32,
        )

        with torch.no_grad():
            logits = self._model(text, vision)[0]
            for index, slot in enumerate(SLOT_IDS):
                if slot not in free_slots:
                    logits[index] = -1e9
            probabilities = torch.softmax(logits, dim=0)
            index = int(torch.argmax(probabilities).item())
        return SLOT_IDS[index], float(probabilities[index].item())
