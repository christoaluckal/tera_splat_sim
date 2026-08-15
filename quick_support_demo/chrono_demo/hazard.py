from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .build_world import add_box, make_contact_material


@dataclass(frozen=True)
class RigidHazard:
    center_x_m: float = 0.13
    center_y_m: float = 0.0
    size_x_m: float = 0.18
    size_y_m: float = 0.24
    height_m: float = 0.13

    @property
    def bounds_xy(self) -> tuple[float, float, float, float]:
        return (
            self.center_x_m - 0.5 * self.size_x_m,
            self.center_x_m + 0.5 * self.size_x_m,
            self.center_y_m - 0.5 * self.size_y_m,
            self.center_y_m + 0.5 * self.size_y_m,
        )

    def intersects_foot(self, position, foot_height_m: float, margin_m: float = 0.01) -> bool:
        xmin, xmax, ymin, ymax = self.bounds_xy
        foot_bottom = float(position.z) - 0.5 * foot_height_m
        return (
            xmin <= float(position.x) <= xmax
            and ymin <= float(position.y) <= ymax
            and foot_bottom <= self.height_m + margin_m
        )


def add_rigid_hazard(system: object, hazard: RigidHazard) -> object:
    return add_box(
        system,
        "offset_rigid_trip_hazard",
        (hazard.size_x_m, hazard.size_y_m, hazard.height_m),
        (hazard.center_x_m, hazard.center_y_m, 0.5 * hazard.height_m),
        make_contact_material(0.95),
        density=1800.0,
        fixed=True,
        color=(0.82, 0.20, 0.06),
    )


def find_hazard_strike(feet, hazard: RigidHazard) -> str | None:
    for leg, body in feet.bodies.items():
        if hazard.intersects_foot(body.GetPos(), feet.foot_height_m):
            return leg
    return None


def opposite_side_support_indices(
    offsets: list[tuple[float, float, float]],
    hazard_center_x_m: float,
) -> set[int]:
    """Return local foot indices on the side opposite the world-x hazard."""
    retained_local_y_sign = 1.0 if hazard_center_x_m >= 0.0 else -1.0
    return {
        index
        for index, (_x, local_y, _z) in enumerate(offsets)
        if local_y * retained_local_y_sign > 0.0
    }


def flat_heightmap(terrain_cfg: dict) -> np.ndarray:
    size_x, size_y = terrain_cfg["pit"]["size_m"]
    spacing = float(terrain_cfg["pit"]["grid_spacing_m"])
    xs = np.arange(-0.5 * size_x, 0.5 * size_x + 0.5 * spacing, spacing)
    ys = np.arange(-0.5 * size_y, 0.5 * size_y + 0.5 * spacing, spacing)
    return np.zeros((len(ys), len(xs)), dtype=np.float32)
