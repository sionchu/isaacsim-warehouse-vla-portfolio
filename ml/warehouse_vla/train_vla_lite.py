from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

ROOT = Path(__file__).resolve().parents[2]
EXTENSION_ROOT = ROOT / "isaacsim_exts" / "warehouse.mission"
sys.path.insert(0, str(EXTENSION_ROOT))

from warehouse_mission.slot_policy import SLOT_IDS  # noqa: E402
from warehouse_mission.vla_model import TinyWarehouseVLA  # noqa: E402

MANUAL_TEMPLATES = (
    "{rack} \uad6c\uc5ed {level}\uce35\uc5d0 \ubc15\uc2a4\ub97c \ubcf4\uad00\ud574",
    "\ubc15\uc2a4\ub97c {rack} \uac04\ubc18 {level}\ub2e8\uc73c\ub85c \uc62e\uaca8\uc918",
    "move the box to rack {rack} level {level}",
    "destination {rack}, shelf {level}",
)
AUTO_TEMPLATES = (
    "\ube44\uc5b4 \uc788\ub294 \uac04\ubc18\uc5d0 \uc790\ub3d9 \ubcf4\uad00\ud574",
    "\uac00\uc7a5 \uc801\uc808\ud55c \ube48 \uc2ac\ub86f\uc73c\ub85c \ubc15\uc2a4\ub97c \uc62e\uaca8",
    "store the box in any available slot",
    "automatic storage",
)


def expert_auto_target(occupancy: list[bool]) -> int:
    free = [index for index, occupied in enumerate(occupancy) if not occupied]
    return sorted(free, key=lambda index: ((index % 5) + 1, index // 5))[0]


def generate_samples(count: int, seed: int):
    rng = random.Random(seed)
    samples = []
    for _ in range(count):
        occupancy = [rng.random() < 0.35 for _ in SLOT_IDS]
        if all(occupancy):
            occupancy[rng.randrange(len(occupancy))] = False
        if rng.random() < 0.65:
            target = rng.randrange(len(SLOT_IDS))
            occupancy[target] = False
            slot = SLOT_IDS[target]
            text = rng.choice(MANUAL_TEMPLATES).format(rack=slot[0], level=slot[1:])
        else:
            target = expert_auto_target(occupancy)
            text = rng.choice(AUTO_TEMPLATES)
        samples.append((text, occupancy, target))
    return samples


def encode_samples(samples, char_to_index, max_length):
    text_tensor = torch.zeros((len(samples), max_length), dtype=torch.long)
    vision_tensor = torch.zeros((len(samples), 1, 2, 5), dtype=torch.float32)
    target_tensor = torch.zeros(len(samples), dtype=torch.long)
    for row, (instruction, occupancy, target) in enumerate(samples):
        for column, character in enumerate(instruction[:max_length]):
            text_tensor[row, column] = char_to_index.get(character, 0)
        vision_tensor[row, 0, 0] = torch.tensor(occupancy[:5], dtype=torch.float32)
        vision_tensor[row, 0, 1] = torch.tensor(occupancy[5:], dtype=torch.float32)
        target_tensor[row] = target
    return TensorDataset(text_tensor, vision_tensor, target_tensor)


def accuracy(model, loader) -> float:
    model.eval()
    device = next(model.parameters()).device
    correct = 0
    total = 0
    with torch.no_grad():
        for text, vision, target in loader:
            text = text.to(device)
            vision = vision.to(device)
            target = target.to(device)
            prediction = model(text, vision).argmax(dim=1)
            correct += int((prediction == target).sum().item())
            total += len(target)
    return correct / max(total, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=5000)
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--output", type=Path, default=ROOT / "models" / "warehouse_vla.pt")
    args = parser.parse_args()

    torch.manual_seed(7)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"device={device}")
    training_samples = generate_samples(args.samples, seed=7)
    validation_samples = generate_samples(max(args.samples // 5, 200), seed=17)
    characters = sorted(
        {character for instruction, _, _ in training_samples for character in instruction}
    )
    char_to_index = {character: index + 1 for index, character in enumerate(characters)}
    max_length = max(len(instruction) for instruction, _, _ in training_samples)

    training_data = encode_samples(training_samples, char_to_index, max_length)
    validation_data = encode_samples(validation_samples, char_to_index, max_length)
    training_loader = DataLoader(training_data, batch_size=128, shuffle=True)
    validation_loader = DataLoader(validation_data, batch_size=256)

    model = TinyWarehouseVLA(len(char_to_index) + 1).to(device)
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
            validation_accuracy = accuracy(model, validation_loader)
            print(
                f"epoch={epoch + 1:02d} "
                f"loss={total_loss / len(training_data):.4f} "
                f"validation_accuracy={validation_accuracy:.3f}"
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
            "slots": SLOT_IDS,
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
