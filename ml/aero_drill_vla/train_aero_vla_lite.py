from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

ROOT = Path(__file__).resolve().parents[2]
EXTENSION_ROOT = ROOT / "isaacsim_exts" / "aero.drill.vla"
sys.path.insert(0, str(EXTENSION_ROOT))

from aero_drill_vla.hole_policy import HOLE_IDS  # noqa: E402
from aero_drill_vla.vla_model import TinyAeroDrillVLA  # noqa: E402

MANUAL_TEMPLATES = (
    "process hole {hole}",
    "dock the drilling unit at {hole}",
    "run the peg in hole alignment on {hole}",
    "{hole} 홀에 드릴 유닛을 도킹해",
)
SEQUENTIAL_TEMPLATES = (
    "process the next pending DRPE hole",
    "continue the ten hole drilling sequence",
    "run the next unfinished hole",
    "다음 DRPE 미완료 홀을 작업해",
)
SAFEST_TEMPLATES = (
    "select the hole with the lowest alignment risk",
    "process the easiest remaining hole",
    "정렬 오차가 가장 작은 홀을 선택해",
)
UPPER_TEMPLATES = ("process the next upper row hole", "continue the upper DRPE row")
LOWER_TEMPLATES = ("process the next lower row hole", "continue the lower DRPE row")


def _first_pending(completed: list[bool], indices=range(10)) -> int | None:
    return next((index for index in indices if not completed[index]), None)


def generate_samples(count: int, seed: int):
    rng = random.Random(seed)
    samples = []
    for _ in range(count):
        completed = [rng.random() < 0.34 for _ in HOLE_IDS]
        if all(completed):
            completed[rng.randrange(10)] = False
        position_errors = [rng.uniform(0.05, 1.40) for _ in HOLE_IDS]
        normal_errors = [rng.uniform(0.05, 1.25) for _ in HOLE_IDS]
        mode = rng.random()
        if mode < 0.52:
            target = rng.randrange(10)
            completed[target] = False
            instruction = rng.choice(MANUAL_TEMPLATES).format(hole=HOLE_IDS[target])
        elif mode < 0.75:
            target = _first_pending(completed)
            instruction = rng.choice(SEQUENTIAL_TEMPLATES)
        elif mode < 0.86:
            pending = [index for index in range(10) if not completed[index]]
            target = min(pending, key=lambda index: position_errors[index] + normal_errors[index])
            instruction = rng.choice(SAFEST_TEMPLATES)
        elif mode < 0.93:
            target = _first_pending(completed, range(5, 10))
            if target is None:
                target = _first_pending(completed)
            instruction = rng.choice(UPPER_TEMPLATES)
        else:
            target = _first_pending(completed, range(0, 5))
            if target is None:
                target = _first_pending(completed)
            instruction = rng.choice(LOWER_TEMPLATES)
        samples.append((instruction, completed, position_errors, normal_errors, target))
    return samples


def encode_samples(samples, char_to_index, max_length):
    text_tensor = torch.zeros((len(samples), max_length), dtype=torch.long)
    vision_tensor = torch.zeros((len(samples), 3, 2, 5), dtype=torch.float32)
    target_tensor = torch.zeros(len(samples), dtype=torch.long)
    for row, (instruction, completed, position_errors, normal_errors, target) in enumerate(samples):
        for column, character in enumerate(instruction[:max_length]):
            text_tensor[row, column] = char_to_index.get(character, 0)
        vision_tensor[row, 0] = torch.tensor(completed, dtype=torch.float32).reshape(2, 5)
        vision_tensor[row, 1] = torch.tensor(position_errors, dtype=torch.float32).reshape(2, 5) / 2.0
        vision_tensor[row, 2] = torch.tensor(normal_errors, dtype=torch.float32).reshape(2, 5) / 2.0
        target_tensor[row] = target
    return TensorDataset(text_tensor, vision_tensor, target_tensor)


def accuracy(model, loader) -> float:
    model.eval()
    device = next(model.parameters()).device
    correct = 0
    total = 0
    with torch.no_grad():
        for text, vision, target in loader:
            prediction = model(text.to(device), vision.to(device)).argmax(dim=1)
            target = target.to(device)
            correct += int((prediction == target).sum().item())
            total += len(target)
    return correct / max(total, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=6000)
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--output", type=Path, default=ROOT / "models" / "aero_drill_vla.pt")
    args = parser.parse_args()

    torch.manual_seed(23)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"device={device}")
    training_samples = generate_samples(args.samples, seed=23)
    validation_samples = generate_samples(max(args.samples // 5, 300), seed=41)
    characters = sorted(
        {character for instruction, *_ in training_samples for character in instruction}
    )
    char_to_index = {character: index + 1 for index, character in enumerate(characters)}
    max_length = max(len(instruction) for instruction, *_ in training_samples)
    training_data = encode_samples(training_samples, char_to_index, max_length)
    validation_data = encode_samples(validation_samples, char_to_index, max_length)
    training_loader = DataLoader(training_data, batch_size=128, shuffle=True)
    validation_loader = DataLoader(validation_data, batch_size=256)

    model = TinyAeroDrillVLA(len(char_to_index) + 1).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=1e-4)
    loss_function = nn.CrossEntropyLoss()
    for epoch in range(args.epochs):
        model.train()
        total_loss = 0.0
        for text, vision, target in training_loader:
            text = text.to(device)
            vision = vision.to(device)
            target = target.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(text, vision)
            loss = loss_function(logits, target)
            loss.backward()
            optimizer.step()
            total_loss += float(loss.item()) * len(target)
        if epoch == 0 or (epoch + 1) % 5 == 0:
            print(
                f"epoch={epoch + 1:02d} loss={total_loss / len(training_data):.4f} "
                f"validation_accuracy={accuracy(model, validation_loader):.3f}"
            )

    validation_accuracy = accuracy(model, validation_loader)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "state_dict": {name: value.detach().cpu() for name, value in model.state_dict().items()},
            "char_to_index": char_to_index,
            "max_length": max_length,
            "embedding_dim": 32,
            "hidden_dim": 48,
            "holes": HOLE_IDS,
            "training_samples": args.samples,
            "epochs": args.epochs,
            "validation_accuracy": validation_accuracy,
        },
        args.output,
    )
    print(f"validation_accuracy={validation_accuracy:.3f}")
    print(f"saved={args.output}")


if __name__ == "__main__":
    main()
