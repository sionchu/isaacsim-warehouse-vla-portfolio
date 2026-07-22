from __future__ import annotations


def _torch_modules():
    import torch
    from torch import nn

    return torch, nn


class TinyWarehouseVLA:
    """Lazy torch wrapper so the extension can still run without a model runtime."""

    def __new__(cls, vocab_size: int, embedding_dim: int = 32, hidden_dim: int = 48):
        _, nn = _torch_modules()

        class _Model(nn.Module):
            def __init__(self) -> None:
                super().__init__()
                self.embedding = nn.Embedding(vocab_size, embedding_dim, padding_idx=0)
                self.text_encoder = nn.GRU(embedding_dim, hidden_dim, batch_first=True)
                self.vision_encoder = nn.Sequential(
                    nn.Conv2d(1, 8, kernel_size=(2, 2)),
                    nn.ReLU(),
                    nn.Flatten(),
                    nn.Linear(32, hidden_dim),
                    nn.ReLU(),
                )
                self.action_head = nn.Sequential(
                    nn.Linear(hidden_dim * 2, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, 10),
                )

            def forward(self, text, vision):
                embedded = self.embedding(text)
                _, hidden = self.text_encoder(embedded)
                language_features = hidden[-1]
                vision_features = self.vision_encoder(vision)
                return self.action_head(
                    __import__("torch").cat((language_features, vision_features), dim=1)
                )

        return _Model()

