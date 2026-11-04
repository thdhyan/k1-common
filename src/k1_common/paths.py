"""Where trained checkpoints live (stdlib only).

Checkpoints are large untracked binaries, so no repository carries them.
Every caller resolves them under one root:

    $K1_MODELS_DIR            (default: ~/k1_models)
      k1_velocity_policy.pt
      mjlab_k1/<run>/model_<iter>.pt
"""
from __future__ import annotations

import os
from pathlib import Path


def models_dir() -> Path:
    return Path(os.environ.get("K1_MODELS_DIR") or Path.home() / "k1_models").expanduser()


def model(rel: str) -> Path:
    """Path of a checkpoint relative to :func:`models_dir` (not checked for existence)."""
    return models_dir() / rel
