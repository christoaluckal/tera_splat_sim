from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def resolve_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def load_yaml(path: str | Path) -> dict[str, Any]:
    with resolve_path(path).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def load_demo_config(path: str | Path = "quick_support_demo/configs/demo.yaml") -> dict[str, Any]:
    demo = load_yaml(path)
    return {
        "world": load_yaml(demo["world_config"]),
        "terrain": load_yaml(demo["terrain_config"]),
        "candidates": load_yaml(demo["candidates_config"]),
        "robots": {name: load_yaml(cfg_path) for name, cfg_path in demo["robots"].items()},
    }

