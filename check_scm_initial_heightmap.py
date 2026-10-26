#!/usr/bin/env python3
"""Round-trip a metric heightfield through native Chrono SCM initialization."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from quick_support_demo.chrono_demo.build_scm_pit import (
    ArrayHeightCourse,
    build_rolling_scm_pit,
    sample_heightmap,
)
from quick_support_demo.chrono_demo.build_world import add_perimeter_floor, build_system
from quick_support_demo.config import load_demo_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--heightmap-npy", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scm-grid-spacing-m", type=float, required=True)
    parser.add_argument("--scm-pit-size-m", type=float, nargs=2, required=True)
    parser.add_argument("--bekker-kphi", type=float, required=True)
    parser.add_argument("--mohr-friction-deg", type=float, required=True)
    parser.add_argument("--maximum-roundtrip-error-m", type=float, default=1.0e-5)
    return parser.parse_args()


def array_sha256(value: np.ndarray) -> str:
    array = np.ascontiguousarray(np.asarray(value, dtype="<f4"))
    descriptor = json.dumps(
        {"dtype": str(array.dtype), "shape": list(array.shape)},
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(descriptor + b"\0" + array.tobytes()).hexdigest()


def main() -> None:
    args = parse_args()
    if args.output_dir.exists():
        raise RuntimeError(f"refusing existing output directory: {args.output_dir}")
    if args.scm_grid_spacing_m <= 0.0 or min(args.scm_pit_size_m) <= 0.0:
        raise ValueError("SCM spacing and pit dimensions must be positive")
    if args.maximum_roundtrip_error_m < 0.0:
        raise ValueError("maximum round-trip error must be non-negative")

    requested = np.load(args.heightmap_npy.resolve(), allow_pickle=False)
    requested = np.asarray(requested, dtype=np.float32)
    cfg = load_demo_config()
    world_cfg = cfg["world"]
    terrain_cfg = cfg["terrain"]
    terrain_cfg["pit"]["size_m"] = [float(value) for value in args.scm_pit_size_m]
    terrain_cfg["pit"]["grid_spacing_m"] = float(args.scm_grid_spacing_m)
    terrain_cfg["soil"]["bekker_kphi"] = float(args.bekker_kphi)
    terrain_cfg["soil"]["mohr_friction_deg"] = float(args.mohr_friction_deg)

    system = build_system(world_cfg)
    add_perimeter_floor(system, world_cfg, terrain_cfg)
    course = ArrayHeightCourse(
        requested,
        float(args.scm_pit_size_m[0]),
        float(args.scm_pit_size_m[1]),
    )
    course.heightmap(terrain_cfg)
    terrain = build_rolling_scm_pit(
        system,
        terrain_cfg,
        course,
        visualization_mesh=False,
    )
    sampled = sample_heightmap(terrain, terrain_cfg)
    absolute_error = np.abs(sampled - requested)
    maximum_error = float(np.max(absolute_error))
    checks = {
        "shape": sampled.shape == requested.shape,
        "finite": bool(np.isfinite(sampled).all()),
        "roundtrip_error": maximum_error <= float(args.maximum_roundtrip_error_m),
    }
    summary = {
        "schema_version": 1,
        "status": "passed" if all(checks.values()) else "failed",
        "heightmap": str(args.heightmap_npy.resolve()),
        "requested_array_sha256": array_sha256(requested),
        "sampled_array_sha256": array_sha256(sampled),
        "shape": list(sampled.shape),
        "maximum_absolute_error_m": maximum_error,
        "mean_absolute_error_m": float(np.mean(absolute_error)),
        "maximum_roundtrip_error_m": float(args.maximum_roundtrip_error_m),
        "checks": checks,
        "soil_parameters": {
            "bekker_kphi": float(args.bekker_kphi),
            "mohr_friction_deg": float(args.mohr_friction_deg),
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    np.save(args.output_dir / "sampled_initial_heightmap_m.npy", sampled)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))
    if not all(checks.values()):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
