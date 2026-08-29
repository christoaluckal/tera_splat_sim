from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import yaml

from quick_support_demo.config import load_demo_config

from .build_scm_pit import build_scm_pit, sample_heightmap
from .build_world import add_perimeter_floor, build_system, make_contact_material
from .chrono_import import import_chrono


chrono, _veh = import_chrono()


CYLINDER_DIAMETER_M = 0.14605
CYLINDER_RADIUS_M = 0.073025
CYLINDER_HEIGHT_M = 0.0508


@dataclass(frozen=True)
class CylinderAction:
    episode_id: str
    mass_kg: float
    center_xy_m: tuple[float, float]
    radius_m: float = CYLINDER_RADIUS_M
    height_m: float = CYLINDER_HEIGHT_M
    start_clearance_m: float = 0.02
    removal: str = "remove_body"


@dataclass(frozen=True)
class LoadingConvergence:
    """Fixed, recorded acceptance rule for a loaded oracle state."""

    min_loading_time_s: float
    max_loading_time_s: float
    linear_speed_threshold_mps: float
    angular_speed_threshold_radps: float
    hold_time_s: float
    required_stable_steps: int


def _length(vector: object) -> float:
    return float((vector.x * vector.x + vector.y * vector.y + vector.z * vector.z) ** 0.5)


def _heightmap_coordinates(terrain_cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    size_x, size_y = (float(value) for value in terrain_cfg["pit"]["size_m"])
    spacing = float(terrain_cfg["pit"]["grid_spacing_m"])
    xs = np.arange(-0.5 * size_x, 0.5 * size_x + 0.5 * spacing, spacing)
    ys = np.arange(-0.5 * size_y, 0.5 * size_y + 0.5 * spacing, spacing)
    return xs, ys


def _build_cylinder(system: object, action: CylinderAction, surface_z_m: float) -> object:
    material = make_contact_material(0.7)
    cylinder = chrono.ChBodyEasyCylinder(
        chrono.ChAxis_Z,
        float(action.radius_m),
        float(action.height_m),
        1000.0,
        True,
        True,
        material,
    )
    cylinder.SetName("validity_cylinder")
    mass = float(action.mass_kg)
    radius = float(action.radius_m)
    height = float(action.height_m)
    cylinder.SetMass(mass)
    cylinder.SetInertiaXX(
        chrono.ChVector3d(
            mass * (3.0 * radius * radius + height * height) / 12.0,
            mass * (3.0 * radius * radius + height * height) / 12.0,
            0.5 * mass * radius * radius,
        )
    )
    cylinder.SetPos(
        chrono.ChVector3d(
            float(action.center_xy_m[0]),
            float(action.center_xy_m[1]),
            float(surface_z_m) + float(action.start_clearance_m) + 0.5 * height,
        )
    )
    cylinder.SetRot(chrono.QUNIT)
    system.Add(cylinder)
    return cylinder


def _add_vertical_guide(system: object, terrain: object, cylinder: object, action: CylinderAction) -> object:
    """Constrain the cylinder to vertical translation in the terrain frame.

    The cylinder remains dynamically loaded by gravity; the guide only removes
    lateral drift and all rotational degrees of freedom. This keeps the
    indentation axis-aligned without changing mass or soil calibration inputs.
    """
    guide_body = chrono.ChBody()
    guide_body.SetName("validity_cylinder_vertical_guide")
    guide_body.SetFixed(True)
    system.Add(guide_body)
    guide = chrono.ChLinkLockPrismatic()
    guide.Initialize(
        cylinder,
        guide_body,
        chrono.ChFramed(
            chrono.ChVector3d(
                float(action.center_xy_m[0]),
                float(action.center_xy_m[1]),
                float(cylinder.GetPos().z),
            ),
            chrono.QUNIT,
        ),
    )
    system.AddLink(guide)
    return guide, guide_body


def _pose_row(body: object, time_s: float, phase: str) -> dict[str, float | str]:
    pos = body.GetPos()
    velocity = body.GetPosDt()
    angular_velocity = body.GetAngVelParent()
    return {
        "time_s": float(time_s),
        "phase": phase,
        "x_m": float(pos.x),
        "y_m": float(pos.y),
        "z_m": float(pos.z),
        "linear_speed_mps": _length(velocity),
        "angular_speed_radps": _length(angular_velocity),
    }


def _write_pose_csv(path: Path, rows: list[dict[str, float | str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_episode(
    output_dir: Path,
    action: CylinderAction,
    world_cfg: dict,
    terrain_cfg: dict,
    initial_heightmap: np.ndarray,
    loaded_heightmap: np.ndarray,
    residual_heightmap: np.ndarray,
    pose_rows: list[dict[str, float | str]],
    loaded_reason: str,
    loading_convergence: dict[str, float | int | bool | None],
    residual_settle_s: float,
    smoke: bool,
    vertical_guide: bool,
    terrain_snapshots: list[tuple[dict[str, float | str | None], np.ndarray]] | None = None,
) -> None:
    xs, ys = _heightmap_coordinates(terrain_cfg)
    valid_mask = np.ones(initial_heightmap.shape, dtype=bool)
    valid_mask[[0, -1], :] = False
    valid_mask[:, [0, -1]] = False
    np.save(output_dir / "initial_heightmap_m.npy", initial_heightmap)
    np.save(output_dir / "loaded_heightmap_m.npy", loaded_heightmap)
    np.save(output_dir / "residual_heightmap_m.npy", residual_heightmap)
    np.save(output_dir / "valid_heightmap_mask.npy", valid_mask)
    _write_pose_csv(output_dir / "object_pose.csv", pose_rows)

    action_data = asdict(action)
    action_data["geometry"] = "right_circular_cylinder"
    action_data["gravity_mps2"] = list(world_cfg["world"]["gravity_mps2"])
    with (output_dir / "action.json").open("w", encoding="utf-8") as file:
        json.dump(action_data, file, indent=2)

    loaded_row = next(row for row in reversed(pose_rows) if row["phase"] == "loaded")
    metrics = {
        "loaded_termination_reason": loaded_reason,
        "loaded_sinkage_m": float(pose_rows[0]["z_m"]) - float(loaded_row["z_m"]),
        "loaded_linear_speed_mps": float(loaded_row["linear_speed_mps"]),
        "loaded_angular_speed_radps": float(loaded_row["angular_speed_radps"]),
        "loaded_convergence": loading_convergence,
        "residual_recovery": {"fixed_duration_s": float(residual_settle_s)},
        "max_loaded_depression_m": float(np.min(loaded_heightmap - initial_heightmap)),
        "max_residual_depression_m": float(np.min(residual_heightmap - initial_heightmap)),
    }
    with (output_dir / "metrics.json").open("w", encoding="utf-8") as file:
        json.dump(metrics, file, indent=2)

    manifest = {
        "schema_version": 1,
        "episode_id": action.episode_id,
        "coordinate_frame": "bed",
        "heightmap": {
            "axis_order": "rows=y_increasing, columns=x_increasing",
            "units": "m",
            "origin_xy_m": [float(xs[0]), float(ys[0])],
            "spacing_m": float(terrain_cfg["pit"]["grid_spacing_m"]),
            "shape": list(initial_heightmap.shape),
            "x_bounds_m": [float(xs[0]), float(xs[-1])],
            "y_bounds_m": [float(ys[0]), float(ys[-1])],
            "valid_mask": "valid_heightmap_mask.npy; one-cell SCM boundary ring excluded",
        },
        "states": {
            "initial": "initial_heightmap_m.npy",
            "loaded": "loaded_heightmap_m.npy",
            "residual": "residual_heightmap_m.npy",
        },
        "action": "action.json",
        "object_pose": "object_pose.csv",
        "metrics": "metrics.json",
        "chrono": {
            "timestep_s": float(world_cfg["world"]["timestep_s"]),
            "terrain_model": "SCM",
            "smoke": bool(smoke),
            "vertical_guide": bool(vertical_guide),
            "loading_convergence": loading_convergence,
            "residual_recovery": {"fixed_duration_s": float(residual_settle_s)},
            "soil_parameters": terrain_cfg["soil"],
        },
    }
    with (output_dir / "manifest.yaml").open("w", encoding="utf-8") as file:
        yaml.safe_dump(manifest, file, sort_keys=False)
    if terrain_snapshots is not None:
        snapshot_dir = output_dir / "terrain_snapshots"
        snapshot_dir.mkdir()
        records: list[dict[str, float | str | None]] = []
        for index, (record, heightmap) in enumerate(terrain_snapshots):
            name = f"snapshot_{index:04d}.npy"
            np.save(snapshot_dir / name, heightmap)
            records.append({**record, "heightmap": name})
        with (snapshot_dir / "manifest.json").open("w", encoding="utf-8") as file:
            json.dump(
                {
                    "schema_version": 1,
                    "source": "SCMTerrain.GetHeight sampled at capture times",
                    "capture_count": len(records),
                    "records": records,
                },
                file,
                indent=2,
            )


def run_cylinder_episode(
    output_dir: Path,
    action: CylinderAction,
    smoke: bool = False,
    residual_settle_s: float = 0.5,
    timestep_s: float | None = None,
    settle_time_s: float | None = None,
    max_loading_time_s: float | None = None,
    loading_linear_speed_threshold_mps: float = 0.006,
    loading_angular_speed_threshold_radps: float | None = None,
    loading_hold_time_s: float = 0.10,
    min_loading_time_s: float = 0.25,
    scm_grid_spacing_m: float | None = None,
    scm_pit_size_m: tuple[float, float] | None = None,
    capture_interval_s: float | None = None,
    vertical_guide: bool = False,
) -> dict:
    cfg = load_demo_config()
    world_cfg = cfg["world"]
    terrain_cfg = cfg["terrain"]
    if smoke:
        world_cfg["world"]["timestep_s"] = 0.001
        world_cfg["world"]["settle_time_s"] = 0.6
        terrain_cfg["pit"]["grid_spacing_m"] = 0.04
    if scm_pit_size_m is not None:
        size_x, size_y = (float(value) for value in scm_pit_size_m)
        if size_x <= 2.0 * float(action.radius_m) or size_y <= 2.0 * float(action.radius_m):
            raise ValueError("scm_pit_size_m must exceed the cylinder diameter in both dimensions")
        terrain_cfg["pit"]["size_m"] = [size_x, size_y]
    if scm_grid_spacing_m is not None:
        if scm_grid_spacing_m <= 0.0:
            raise ValueError("scm_grid_spacing_m must be positive")
        terrain_cfg["pit"]["grid_spacing_m"] = float(scm_grid_spacing_m)
    if timestep_s is not None:
        if timestep_s <= 0.0:
            raise ValueError("timestep_s must be positive")
        world_cfg["world"]["timestep_s"] = float(timestep_s)
    if settle_time_s is not None and max_loading_time_s is not None:
        raise ValueError("settle_time_s and max_loading_time_s are aliases; supply only one")
    if settle_time_s is not None:
        if settle_time_s <= 0.0:
            raise ValueError("settle_time_s must be positive")
        max_loading_time_s = float(settle_time_s)
    if max_loading_time_s is None:
        max_loading_time_s = float(world_cfg["world"]["settle_time_s"])
    if max_loading_time_s <= 0.0:
        raise ValueError("max_loading_time_s must be positive")
    if loading_linear_speed_threshold_mps <= 0.0:
        raise ValueError("loading_linear_speed_threshold_mps must be positive")
    if loading_angular_speed_threshold_radps is None:
        loading_angular_speed_threshold_radps = float(world_cfg["world"]["settle_ang_velocity_radps"])
    if loading_angular_speed_threshold_radps <= 0.0:
        raise ValueError("loading_angular_speed_threshold_radps must be positive")
    if loading_hold_time_s <= 0.0 or min_loading_time_s < 0.0:
        raise ValueError("loading_hold_time_s must be positive and min_loading_time_s must be non-negative")
    if min_loading_time_s + loading_hold_time_s > max_loading_time_s:
        raise ValueError("max_loading_time_s must allow min_loading_time_s plus loading_hold_time_s")
    if capture_interval_s is not None and capture_interval_s <= 0.0:
        raise ValueError("capture_interval_s must be positive")
    system = build_system(world_cfg)
    add_perimeter_floor(system, world_cfg, terrain_cfg)
    terrain = build_scm_pit(system, terrain_cfg, visualization_mesh=False)
    initial_heightmap = sample_heightmap(terrain, terrain_cfg)
    cylinder = _build_cylinder(system, action, float(terrain_cfg["pit"]["top_elevation_m"]))
    guide, guide_body = _add_vertical_guide(system, terrain, cylinder, action) if vertical_guide else (None, None)

    dt = float(world_cfg["world"]["timestep_s"])
    convergence = LoadingConvergence(
        min_loading_time_s=float(min_loading_time_s),
        max_loading_time_s=float(max_loading_time_s),
        linear_speed_threshold_mps=float(loading_linear_speed_threshold_mps),
        angular_speed_threshold_radps=float(loading_angular_speed_threshold_radps),
        hold_time_s=float(loading_hold_time_s),
        required_stable_steps=max(1, int(np.ceil(float(loading_hold_time_s) / dt))),
    )
    max_steps = int(np.ceil(convergence.max_loading_time_s / dt))
    stable_steps = 0
    gate_window_start_s: float | None = None
    accepted_time_s: float | None = None
    pose_rows = [_pose_row(cylinder, float(system.GetChTime()), "initial")]
    terrain_snapshots: list[tuple[dict[str, float | str | None], np.ndarray]] | None = None
    next_capture_time = float("inf")
    if capture_interval_s is not None:
        terrain_snapshots = [
            (
                {
                    "time_s": 0.0,
                    "phase": "initial",
                    "body_x_m": float(pose_rows[0]["x_m"]),
                    "body_y_m": float(pose_rows[0]["y_m"]),
                    "body_z_m": float(pose_rows[0]["z_m"]),
                },
                initial_heightmap.copy(),
            )
        ]
        next_capture_time = float(capture_interval_s)
    loaded_reason = "timeout"
    for _ in range(max_steps):
        terrain.Synchronize(system.GetChTime())
        system.DoStepDynamics(dt)
        terrain.Advance(dt)
        row = _pose_row(cylinder, float(system.GetChTime()), "loaded")
        pose_rows.append(row)
        if terrain_snapshots is not None and float(system.GetChTime()) + 1.0e-12 >= next_capture_time:
            terrain_snapshots.append(
                (
                    {
                        "time_s": float(system.GetChTime()),
                        "phase": "loaded",
                        "body_x_m": float(row["x_m"]),
                        "body_y_m": float(row["y_m"]),
                        "body_z_m": float(row["z_m"]),
                    },
                    sample_heightmap(terrain, terrain_cfg),
                )
            )
            next_capture_time += float(capture_interval_s)
        time_s = float(system.GetChTime())
        is_below_threshold = (
            time_s >= convergence.min_loading_time_s
            and float(row["linear_speed_mps"]) <= convergence.linear_speed_threshold_mps
            and float(row["angular_speed_radps"]) <= convergence.angular_speed_threshold_radps
        )
        if is_below_threshold:
            if stable_steps == 0:
                gate_window_start_s = time_s
            stable_steps += 1
            if stable_steps >= convergence.required_stable_steps:
                loaded_reason = "converged_speed_hold"
                accepted_time_s = time_s
                break
        else:
            stable_steps = 0
            gate_window_start_s = None
    loaded_heightmap = sample_heightmap(terrain, terrain_cfg)
    loaded_row = pose_rows[-1]
    loading_convergence = {
        "accepted": loaded_reason == "converged_speed_hold",
        "min_loading_time_s": convergence.min_loading_time_s,
        "max_loading_time_s": convergence.max_loading_time_s,
        "linear_speed_threshold_mps": convergence.linear_speed_threshold_mps,
        "angular_speed_threshold_radps": convergence.angular_speed_threshold_radps,
        "hold_time_s": convergence.hold_time_s,
        "required_stable_steps": convergence.required_stable_steps,
        "gate_window_start_s": gate_window_start_s if accepted_time_s is not None else None,
        "accepted_time_s": accepted_time_s,
        "final_sample_time_s": float(loaded_row["time_s"]),
        "final_linear_speed_mps": float(loaded_row["linear_speed_mps"]),
        "final_angular_speed_radps": float(loaded_row["angular_speed_radps"]),
    }
    if terrain_snapshots is not None:
        terrain_snapshots.append(
            (
                {
                    "time_s": float(loaded_row["time_s"]),
                    "phase": "loaded_accepted" if accepted_time_s is not None else "loaded_timeout",
                    "body_x_m": float(loaded_row["x_m"]),
                    "body_y_m": float(loaded_row["y_m"]),
                    "body_z_m": float(loaded_row["z_m"]),
                },
                loaded_heightmap.copy(),
            )
        )

    if guide is not None:
        system.Remove(guide)
    if guide_body is not None:
        system.Remove(guide_body)
    system.Remove(cylinder)
    for _ in range(int(float(residual_settle_s) / dt)):
        terrain.Synchronize(system.GetChTime())
        system.DoStepDynamics(dt)
        terrain.Advance(dt)
        if terrain_snapshots is not None and float(system.GetChTime()) + 1.0e-12 >= next_capture_time:
            terrain_snapshots.append(
                (
                    {
                        "time_s": float(system.GetChTime()),
                        "phase": "post_removal",
                        "body_x_m": None,
                        "body_y_m": None,
                        "body_z_m": None,
                    },
                    sample_heightmap(terrain, terrain_cfg),
                )
            )
            next_capture_time += float(capture_interval_s)
    residual_heightmap = sample_heightmap(terrain, terrain_cfg)
    if terrain_snapshots is not None:
        terrain_snapshots.append(
            (
                {
                    "time_s": float(system.GetChTime()),
                    "phase": "residual_fixed_time",
                    "body_x_m": None,
                    "body_y_m": None,
                    "body_z_m": None,
                },
                residual_heightmap.copy(),
            )
        )
    output_dir.mkdir(parents=True, exist_ok=False)
    _write_episode(
        output_dir,
        action,
        world_cfg,
        terrain_cfg,
        initial_heightmap,
        loaded_heightmap,
        residual_heightmap,
        pose_rows,
        loaded_reason,
        loading_convergence,
        residual_settle_s,
        smoke,
        vertical_guide,
        terrain_snapshots,
    )
    return {
        "output_dir": str(output_dir),
        "loaded_termination_reason": loaded_reason,
        "loaded_convergence_accepted": bool(loading_convergence["accepted"]),
    }
