#!/usr/bin/env python3
"""Render a Chrono A0 BayesOpt target as viridis elevation/deformation DEMs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import yaml


STATES = ("initial", "loaded", "residual")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chrono-episode", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    episode = args.chrono_episode.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite existing output directory: {output_dir}")

    with (episode / "manifest.yaml").open("r", encoding="utf-8") as file:
        manifest = yaml.safe_load(file)
    with (episode / "action.json").open("r", encoding="utf-8") as file:
        action = json.load(file)
    spec = manifest["heightmap"]
    mask = np.load(episode / "valid_heightmap_mask.npy").astype(bool)
    heightmaps = {state: np.load(episode / f"{state}_heightmap_m.npy") for state in STATES}
    shape = tuple(int(value) for value in spec["shape"])
    if any(field.shape != shape for field in heightmaps.values()) or mask.shape != shape:
        raise ValueError("Chrono heightmap or mask does not match the manifest shape")

    elevation_values = np.concatenate([field[mask] for field in heightmaps.values()])
    elevation_min_m = float(np.min(elevation_values))
    elevation_max_m = float(np.max(elevation_values))
    if elevation_min_m == elevation_max_m:
        elevation_max_m = elevation_min_m + 1e-9
    depression_mm = {state: (heightmaps["initial"] - field) * 1000.0 for state, field in heightmaps.items()}
    depression_max_mm = max(float(np.max(field[mask])) for field in depression_mm.values())
    depression_max_mm = max(depression_max_mm, 1e-3)

    spacing = float(spec["spacing_m"])
    x0, y0 = (float(value) for value in spec["origin_xy_m"])
    rows, cols = shape
    extent = (x0 - 0.5 * spacing, x0 + (cols - 0.5) * spacing, y0 - 0.5 * spacing, y0 + (rows - 0.5) * spacing)
    titles = {"initial": "Initial", "loaded": "Loaded / during", "residual": "Residual / after"}

    figure, axes = plt.subplots(2, 3, figsize=(15, 9), layout="constrained", sharex=True, sharey=True)
    elevation_image = depression_image = None
    for column, state in enumerate(STATES):
        elevation_image = axes[0, column].imshow(
            np.ma.array(heightmaps[state], mask=~mask),
            extent=extent,
            origin="lower",
            cmap="viridis",
            vmin=elevation_min_m,
            vmax=elevation_max_m,
            interpolation="nearest",
        )
        depression_image = axes[1, column].imshow(
            np.ma.array(depression_mm[state], mask=~mask),
            extent=extent,
            origin="lower",
            cmap="viridis",
            vmin=0.0,
            vmax=depression_max_mm,
            interpolation="nearest",
        )
        for row in range(2):
            axes[row, column].add_patch(
                Circle(action["center_xy_m"], action["radius_m"], fill=False, color="white", linewidth=1.1)
            )
            axes[row, column].set_aspect("equal")
            axes[row, column].set_title(titles[state])
        axes[1, column].set_xlabel("bed x (m)")
    axes[0, 0].set_ylabel("elevation\nbed y (m)")
    axes[1, 0].set_ylabel("depression from initial\nbed y (m)")
    figure.colorbar(elevation_image, ax=axes[0], shrink=0.86, label="surface elevation (m)")
    figure.colorbar(depression_image, ax=axes[1], shrink=0.86, label="downward deformation from initial (mm)")
    figure.suptitle(
        f"Chrono SCM BayesOpt target: {manifest['episode_id']} — {action['mass_kg']:.1f} kg cylinder, "
        f"{spacing * 1000:g} mm grid, {int(mask.sum())} valid cells",
        fontsize=13,
    )

    output_dir.mkdir(parents=True)
    figure.savefig(output_dir / "chrono_target_dem_viridis.png", dpi=200)
    plt.close(figure)
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as file:
        json.dump(
            {
                "schema_version": 1,
                "chrono_episode": str(episode),
                "coordinate_frame": manifest["coordinate_frame"],
                "heightmap": spec,
                "states": list(STATES),
                "mask": "valid_heightmap_mask.npy from Chrono episode",
                "render": {
                    "image": "chrono_target_dem_viridis.png",
                    "elevation_colormap": "viridis",
                    "elevation_range_m": [elevation_min_m, elevation_max_m],
                    "depression_colormap": "viridis",
                    "depression_range_mm": [0.0, depression_max_mm],
                },
            },
            file,
            indent=2,
        )
    print(output_dir)


if __name__ == "__main__":
    main()
