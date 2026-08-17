#!/usr/bin/env python3
"""Export native and display-upsampled SCM/Genesis height-field meshes.

The display mesh uses cubic interpolation; it improves visual smoothness but
does not add simulation detail beyond the source Chrono grid.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import RectBivariateSpline


STATES = ("initial", "loaded", "residual")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chrono-episode", type=Path, required=True)
    parser.add_argument("--genesis-bridge", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--display-spacing-m", type=float, default=0.005)
    return parser.parse_args()


def write_mesh_ply(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    with path.open("w", encoding="ascii") as file:
        file.write("ply\nformat ascii 1.0\n")
        file.write(f"element vertex {vertices.shape[0]}\nproperty float x\nproperty float y\nproperty float z\n")
        file.write(f"element face {faces.shape[0]}\nproperty list uchar int vertex_indices\nend_header\n")
        np.savetxt(file, vertices, fmt="%.8f %.8f %.8f")
        np.savetxt(file, faces, fmt="3 %d %d %d")


def valid_rectangle(mask: np.ndarray) -> tuple[slice, slice]:
    rows, cols = np.where(mask)
    if rows.size == 0:
        raise ValueError("Common mask is empty")
    row_slice = slice(int(rows.min()), int(rows.max()) + 1)
    col_slice = slice(int(cols.min()), int(cols.max()) + 1)
    if not np.all(mask[row_slice, col_slice]):
        raise ValueError("Common mask is not a rectangle; refusing to bridge invalid holes")
    return row_slice, col_slice


def triangulate(heightmap: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    vertices = np.column_stack((xx.ravel(), yy.ravel(), heightmap.ravel())).astype(np.float32)
    rows, cols = heightmap.shape
    # Index of each top-left node in the flattened row-major vertex layout.
    top_left = (np.arange(rows - 1)[:, None] * cols + np.arange(cols - 1)[None, :]).astype(np.int64)
    faces = np.empty((2 * top_left.size, 3), dtype=np.int64)
    faces[0::2] = np.column_stack((top_left.ravel(), (top_left + 1).ravel(), (top_left + cols).ravel()))
    faces[1::2] = np.column_stack(((top_left + 1).ravel(), (top_left + cols + 1).ravel(), (top_left + cols).ravel()))
    return vertices, faces


def display_axes(source: np.ndarray, spacing: float) -> np.ndarray:
    if spacing <= 0:
        raise ValueError("display spacing must be positive")
    intervals = max(1, int(round((source[-1] - source[0]) / spacing)))
    return np.linspace(source[0], source[-1], intervals + 1, dtype=np.float32)


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
    spec = chrono_manifest["heightmap"]
    if spec != genesis_manifest["heightmap"]:
        raise ValueError("Chrono and Genesis grids differ")
    spacing = float(spec["spacing_m"])
    rows, cols = (int(value) for value in spec["shape"])
    x0, y0 = (float(value) for value in spec["origin_xy_m"])
    xs_full = x0 + spacing * np.arange(cols, dtype=np.float32)
    ys_full = y0 + spacing * np.arange(rows, dtype=np.float32)
    mask = np.load(chrono_dir / "valid_heightmap_mask.npy").astype(bool) & np.load(genesis_dir / "valid_heightmap_mask.npy").astype(bool)
    row_slice, col_slice = valid_rectangle(mask)
    xs = xs_full[col_slice]
    ys = ys_full[row_slice]
    xs_display = display_axes(xs, args.display_spacing_m)
    ys_display = display_axes(ys, args.display_spacing_m)
    output_dir.mkdir(parents=True)
    artifacts: dict[str, dict[str, int]] = {}
    for source_name, source_dir in (("scm", chrono_dir), ("genesis", genesis_dir)):
        for state in STATES:
            source = np.load(source_dir / f"{state}_heightmap_m.npy")[row_slice, col_slice]
            if not np.isfinite(source).all():
                raise ValueError(f"Non-finite {source_name} {state} heightmap in valid rectangle")
            native_vertices, native_faces = triangulate(source, xs, ys)
            spline = RectBivariateSpline(ys, xs, source, kx=3, ky=3)
            display = spline(ys_display, xs_display).astype(np.float32)
            display_vertices, display_faces = triangulate(display, xs_display, ys_display)
            write_mesh_ply(output_dir / f"{source_name}_{state}_native_{spacing * 1000:g}mm.ply", native_vertices, native_faces)
            write_mesh_ply(output_dir / f"{source_name}_{state}_display_{args.display_spacing_m * 1000:g}mm.ply", display_vertices, display_faces)
            artifacts[f"{source_name}_{state}"] = {
                "native_vertices": int(native_vertices.shape[0]),
                "native_faces": int(native_faces.shape[0]),
                "display_vertices": int(display_vertices.shape[0]),
                "display_faces": int(display_faces.shape[0]),
            }
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as file:
        json.dump(
            {
                "schema_version": 1,
                "chrono_episode": str(chrono_dir),
                "genesis_bridge": str(genesis_dir),
                "coordinate_frame": chrono_manifest["coordinate_frame"],
                "source_grid_spacing_m": spacing,
                "display_grid_spacing_m": args.display_spacing_m,
                "interpolation": "RectBivariateSpline cubic; display-only, no new simulation information",
                "common_valid_rectangle": {"rows": [row_slice.start, row_slice.stop - 1], "cols": [col_slice.start, col_slice.stop - 1]},
                "artifacts": artifacts,
            },
            file,
            indent=2,
        )
    print(output_dir)


if __name__ == "__main__":
    main()
