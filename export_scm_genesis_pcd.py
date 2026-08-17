#!/usr/bin/env python3
"""Export directly comparable Chrono SCM and Genesis heightmap PCDs.

The main PCDs use the exact Chrono grid and valid mask, making them suitable
for direct overlays in CloudCompare, Open3D, or PCL viewers.  Genesis raw MPM
particle PCDs are exported separately when the bridge retains their binary
PLY source.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


STATES = ("initial", "loaded", "residual")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chrono-episode", type=Path, required=True)
    parser.add_argument("--genesis-bridge", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--no-raw-genesis", action="store_true", help="Skip raw MPM particle PCD export.")
    return parser.parse_args()


def write_pcd(path: Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError(f"PCD points for {path} must be finite [N, 3]")
    with path.open("w", encoding="ascii") as file:
        file.write("# .PCD v0.7 - Point Cloud Data file format\n")
        file.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        file.write(f"WIDTH {points.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {points.shape[0]}\nDATA ascii\n")
        np.savetxt(file, points, fmt="%.8f %.8f %.8f")


def grid_points(heightmap: np.ndarray, valid_mask: np.ndarray, heightmap_spec: dict) -> np.ndarray:
    rows, cols = (int(value) for value in heightmap_spec["shape"])
    if heightmap.shape != (rows, cols) or valid_mask.shape != (rows, cols):
        raise ValueError("Heightmap/mask shape does not match the Chrono manifest")
    spacing = float(heightmap_spec["spacing_m"])
    x0, y0 = (float(value) for value in heightmap_spec["origin_xy_m"])
    xs = x0 + spacing * np.arange(cols, dtype=np.float32)
    ys = y0 + spacing * np.arange(rows, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    keep = valid_mask.astype(bool) & np.isfinite(heightmap)
    return np.column_stack((xx[keep], yy[keep], heightmap[keep])).astype(np.float32)


def read_xyz_binary_ply(path: Path) -> np.ndarray:
    """Read x/y/z from the simple binary-little-endian float PLY written here."""
    raw = path.read_bytes()
    marker = b"end_header\n"
    header_end = raw.find(marker)
    if header_end < 0:
        raise ValueError(f"No PLY header terminator in {path}")
    header = raw[: header_end + len(marker)].decode("ascii")
    if "format binary_little_endian 1.0" not in header:
        raise ValueError(f"Expected binary_little_endian PLY: {path}")
    vertex_line = next(line for line in header.splitlines() if line.startswith("element vertex "))
    count = int(vertex_line.split()[-1])
    properties = [line.split()[-1] for line in header.splitlines() if line.startswith("property ")]
    if not {"x", "y", "z"}.issubset(properties):
        raise ValueError(f"PLY lacks x/y/z: {path}")
    if any(not line.startswith("property float ") for line in header.splitlines() if line.startswith("property ")):
        raise ValueError(f"Unsupported non-float PLY properties: {path}")
    values = np.frombuffer(raw, dtype="<f4", offset=header_end + len(marker), count=count * len(properties))
    if values.size != count * len(properties):
        raise ValueError(f"Truncated PLY data: {path}")
    values = values.reshape(count, len(properties))
    return values[:, [properties.index("x"), properties.index("y"), properties.index("z")]].copy()


def main() -> None:
    args = parse_args()
    chrono_dir = args.chrono_episode.resolve()
    genesis_dir = args.genesis_bridge.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite existing output directory: {output_dir}")
    with (chrono_dir / "manifest.yaml").open("r", encoding="utf-8") as file:
        chrono_manifest = yaml.safe_load(file)
    with (genesis_dir / "manifest.json").open("r", encoding="utf-8") as file:
        genesis_manifest = json.load(file)
    if chrono_manifest["heightmap"] != genesis_manifest["heightmap"]:
        raise ValueError("Chrono and Genesis manifests do not describe the same comparison grid")
    chrono_mask = np.load(chrono_dir / "valid_heightmap_mask.npy").astype(bool)
    genesis_mask = np.load(genesis_dir / "valid_heightmap_mask.npy").astype(bool)
    common_mask = chrono_mask & genesis_mask
    output_dir.mkdir(parents=True)
    counts: dict[str, int] = {}
    for state in STATES:
        chrono_points = grid_points(np.load(chrono_dir / f"{state}_heightmap_m.npy"), common_mask, chrono_manifest["heightmap"])
        genesis_points = grid_points(np.load(genesis_dir / f"{state}_heightmap_m.npy"), common_mask, chrono_manifest["heightmap"])
        write_pcd(output_dir / f"scm_{state}_chrono_grid.pcd", chrono_points)
        write_pcd(output_dir / f"genesis_{state}_chrono_grid.pcd", genesis_points)
        counts[f"scm_{state}"] = int(chrono_points.shape[0])
        counts[f"genesis_{state}"] = int(genesis_points.shape[0])
    if not args.no_raw_genesis:
        raw_dir = genesis_dir / "genesis_raw"
        raw_names = {"initial": "particles_initial_mpm.ply", "loaded": "particles_loaded_mpm.ply", "residual": "particles_final_mpm.ply"}
        for state, name in raw_names.items():
            source = raw_dir / name
            if source.is_file():
                raw_points = read_xyz_binary_ply(source)
                write_pcd(output_dir / f"genesis_{state}_raw_mpm_particles.pcd", raw_points)
                counts[f"genesis_{state}_raw"] = int(raw_points.shape[0])
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as file:
        json.dump(
            {
                "schema_version": 1,
                "chrono_episode": str(chrono_dir),
                "genesis_bridge": str(genesis_dir),
                "coordinate_frame": chrono_manifest["coordinate_frame"],
                "heightmap": chrono_manifest["heightmap"],
                "mask": "common Chrono/Genesis valid_heightmap_mask intersection",
                "files": counts,
            },
            file,
            indent=2,
        )
    print(output_dir)


if __name__ == "__main__":
    main()
