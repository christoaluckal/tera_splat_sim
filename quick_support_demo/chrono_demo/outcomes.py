from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class SupportOutcome:
    robot: str
    candidate_pose: str
    foot_sinkage_m: np.ndarray
    body_roll_rad: float
    body_pitch_rad: float
    com_height_change_m: float
    initial_heightmap_m: np.ndarray
    loaded_heightmap_m: np.ndarray
    residual_heightmap_m: np.ndarray
    runtime_s: float
    selected_candidate: str | None = None
    total_cost: float | None = None

    @property
    def max_sinkage_m(self) -> float:
        return float(np.max(self.foot_sinkage_m))

    @property
    def mean_sinkage_m(self) -> float:
        return float(np.mean(self.foot_sinkage_m))

    @property
    def max_abs_tilt_rad(self) -> float:
        return float(max(abs(self.body_roll_rad), abs(self.body_pitch_rad)))

    def to_json_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["foot_sinkage_m"] = self.foot_sinkage_m.tolist()
        data["initial_heightmap_shape"] = list(self.initial_heightmap_m.shape)
        data["loaded_heightmap_shape"] = list(self.loaded_heightmap_m.shape)
        data["residual_heightmap_shape"] = list(self.residual_heightmap_m.shape)
        data.pop("initial_heightmap_m")
        data.pop("loaded_heightmap_m")
        data.pop("residual_heightmap_m")
        data["max_sinkage_m"] = self.max_sinkage_m
        data["mean_sinkage_m"] = self.mean_sinkage_m
        data["max_abs_tilt_rad"] = self.max_abs_tilt_rad
        return data


def save_heightmaps(outcome: SupportOutcome, output_dir: Path) -> None:
    np.save(output_dir / "initial_heightmap_m.npy", outcome.initial_heightmap_m)
    np.save(output_dir / "loaded_heightmap_m.npy", outcome.loaded_heightmap_m)
    np.save(output_dir / "residual_heightmap_m.npy", outcome.residual_heightmap_m)

