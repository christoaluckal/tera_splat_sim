from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .build_world import add_box, make_contact_material
from .chrono_import import import_chrono


chrono, _veh = import_chrono()


@dataclass(frozen=True)
class RigidPad:
    center_x_m: float
    center_y_m: float
    size_x_m: float
    size_y_m: float
    height_m: float

    @property
    def bounds_xy(self) -> tuple[float, float, float, float]:
        return (
            self.center_x_m - 0.5 * self.size_x_m,
            self.center_x_m + 0.5 * self.size_x_m,
            self.center_y_m - 0.5 * self.size_y_m,
            self.center_y_m + 0.5 * self.size_y_m,
        )

    def contains_xy(self, x_m: float, y_m: float) -> bool:
        xmin, xmax, ymin, ymax = self.bounds_xy
        return xmin <= x_m <= xmax and ymin <= y_m <= ymax


@dataclass(frozen=True)
class DifficultCourse:
    pads: tuple[RigidPad, ...]

    def height_at(self, x_m: float, y_m: float) -> float:
        return max(
            (pad.height_m for pad in self.pads if pad.contains_xy(x_m, y_m)),
            default=0.0,
        )

    def heightmap(self, terrain_cfg: dict) -> np.ndarray:
        size_x, size_y = terrain_cfg["pit"]["size_m"]
        spacing = float(terrain_cfg["pit"]["grid_spacing_m"])
        xs = np.arange(-0.5 * size_x, 0.5 * size_x + 0.5 * spacing, spacing)
        ys = np.arange(-0.5 * size_y, 0.5 * size_y + 0.5 * spacing, spacing)
        result = np.zeros((len(ys), len(xs)), dtype=np.float32)
        for pad in self.pads:
            xmin, xmax, ymin, ymax = pad.bounds_xy
            x_mask = (xs >= xmin) & (xs <= xmax)
            y_mask = (ys >= ymin) & (ys <= ymax)
            result[np.ix_(y_mask, x_mask)] = np.maximum(
                result[np.ix_(y_mask, x_mask)], pad.height_m
            )
        return result


@dataclass(frozen=True)
class RollingCourse:
    size_x_m: float = 1.2
    size_y_m: float = 1.2

    def elevation(self, x_m, y_m):
        x = np.asarray(x_m, dtype=float)
        y = np.asarray(y_m, dtype=float)
        taper_x = np.clip(1.0 - (x / (0.5 * self.size_x_m)) ** 8, 0.0, 1.0)
        taper_y = np.clip(1.0 - (y / (0.5 * self.size_y_m)) ** 8, 0.0, 1.0)
        taper = taper_x * taper_y
        near_hill = 0.072 * np.exp(-((x - 0.10) / 0.34) ** 2 - ((y + 0.30) / 0.17) ** 2)
        center_valley = -0.058 * np.exp(-((x + 0.05) / 0.30) ** 2 - ((y - 0.02) / 0.19) ** 2)
        far_hill = 0.060 * np.exp(-((x + 0.12) / 0.32) ** 2 - ((y - 0.36) / 0.16) ** 2)
        cross_slope = 0.020 * np.sin(np.pi * x / 0.60) * np.sin(np.pi * (y + 0.60) / 1.20)
        return taper * (near_hill + center_valley + far_hill + cross_slope)

    def height_at(self, x_m: float, y_m: float) -> float:
        if abs(x_m) > 0.5 * self.size_x_m or abs(y_m) > 0.5 * self.size_y_m:
            return 0.0
        return float(self.elevation(x_m, y_m))

    def heightmap(self, terrain_cfg: dict) -> np.ndarray:
        size_x, size_y = terrain_cfg["pit"]["size_m"]
        spacing = float(terrain_cfg["pit"]["grid_spacing_m"])
        xs = np.arange(-0.5 * size_x, 0.5 * size_x + 0.5 * spacing, spacing)
        ys = np.arange(-0.5 * size_y, 0.5 * size_y + 0.5 * spacing, spacing)
        xx, yy = np.meshgrid(xs, ys)
        return np.asarray(self.elevation(xx, yy), dtype=np.float32)


def default_difficult_course() -> DifficultCourse:
    return DifficultCourse(
        pads=(
            RigidPad(0.13, -0.28, 0.30, 0.34, 0.085),
            RigidPad(-0.13, 0.12, 0.30, 0.34, 0.070),
            RigidPad(0.08, 0.47, 0.44, 0.18, 0.055),
        )
    )


def default_rolling_course() -> RollingCourse:
    return RollingCourse()


def add_difficult_course(system: object, course: DifficultCourse) -> list[object]:
    material = make_contact_material(0.82)
    bodies = []
    colors = ((0.86, 0.44, 0.08), (0.18, 0.48, 0.68), (0.72, 0.62, 0.12))
    for index, (pad, color) in enumerate(zip(course.pads, colors, strict=True)):
        bodies.append(
            add_box(
                system,
                f"difficult_course_pad_{index}",
                (pad.size_x_m, pad.size_y_m, pad.height_m),
                (pad.center_x_m, pad.center_y_m, 0.5 * pad.height_m),
                material,
                density=1800.0,
                fixed=True,
                color=color,
            )
        )
    return bodies


def add_rolling_course(system: object, course: RollingCourse, terrain_cfg: dict) -> object:
    mesh = build_rolling_mesh(course, terrain_cfg)
    body = chrono.ChBody()
    body.SetName("rolling_rigid_heightfield")
    body.SetFixed(True)
    body.AddCollisionShape(
        chrono.ChCollisionShapeTriangleMesh(
            make_contact_material(0.82), mesh, True, False, 0.002
        )
    )
    body.EnableCollision(True)
    system.Add(body)
    return body


def build_rolling_mesh(course: RollingCourse, terrain_cfg: dict) -> object:
    heightmap = course.heightmap(terrain_cfg)
    rows, cols = heightmap.shape
    xs = np.linspace(-0.5 * course.size_x_m, 0.5 * course.size_x_m, cols)
    ys = np.linspace(-0.5 * course.size_y_m, 0.5 * course.size_y_m, rows)
    mesh = chrono.ChTriangleMeshConnected()
    for row in range(rows - 1):
        for col in range(cols - 1):
            p00 = chrono.ChVector3d(xs[col], ys[row], float(heightmap[row, col]))
            p10 = chrono.ChVector3d(xs[col + 1], ys[row], float(heightmap[row, col + 1]))
            p11 = chrono.ChVector3d(xs[col + 1], ys[row + 1], float(heightmap[row + 1, col + 1]))
            p01 = chrono.ChVector3d(xs[col], ys[row + 1], float(heightmap[row + 1, col]))
            mesh.AddTriangle(p00, p10, p11)
            mesh.AddTriangle(p00, p11, p01)
    return mesh


def support_plane_attitude(
    foot_positions_body: dict[str, np.ndarray],
    body_xy_m: tuple[float, float],
    yaw_rad: float,
    course: DifficultCourse | RollingCourse,
    max_tilt_rad: float,
) -> tuple[float, float, float]:
    """Return local roll, pitch, and center support elevation."""
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    samples = []
    heights = []
    for position in foot_positions_body.values():
        local_x, local_y = float(position[0]), float(position[1])
        world_x = body_xy_m[0] + c * local_x - s * local_y
        world_y = body_xy_m[1] + s * local_x + c * local_y
        samples.append((local_x, local_y, 1.0))
        heights.append(course.height_at(world_x, world_y))

    slope_x, slope_y, center_height = np.linalg.lstsq(
        np.asarray(samples, dtype=float), np.asarray(heights, dtype=float), rcond=None
    )[0]
    roll = np.clip(np.arctan(slope_y), -max_tilt_rad, max_tilt_rad)
    pitch = np.clip(-np.arctan(slope_x), -max_tilt_rad, max_tilt_rad)
    return float(roll), float(pitch), float(center_height)


def terrain_adjusted_foot_targets(
    foot_positions_body: dict[str, np.ndarray],
    nominal_foot_z_m: float,
    body_position_world: np.ndarray,
    yaw_rad: float,
    roll_rad: float,
    pitch_rad: float,
    course: DifficultCourse | RollingCourse,
    foot_height_m: float,
) -> dict[str, np.ndarray]:
    """Map gait feet to exact rigid-surface heights in the tilted body frame."""
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    rotation = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ]
    ) @ np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ]
    ) @ np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ]
    )

    targets = {}
    for leg, position in foot_positions_body.items():
        local_x, local_y = float(position[0]), float(position[1])
        world_x = body_position_world[0] + cy * local_x - sy * local_y
        world_y = body_position_world[1] + sy * local_x + cy * local_y
        swing_clearance = max(float(position[2]) - nominal_foot_z_m, 0.0)
        world_z = course.height_at(world_x, world_y) + 0.5 * foot_height_m + swing_clearance
        world_target = np.array([world_x, world_y, world_z], dtype=float)
        targets[leg] = rotation.T @ (world_target - body_position_world)
    return targets
