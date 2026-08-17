from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Circle


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render Chrono and Genesis initial/loaded/residual heightmap states.")
    parser.add_argument("--chrono-episode", type=Path, required=True)
    parser.add_argument("--genesis-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def load_chrono_states(path: Path) -> tuple[dict, dict, np.ndarray, list[np.ndarray]]:
    with (path / "manifest.yaml").open("r", encoding="utf-8") as file:
        manifest = yaml.safe_load(file)
    with (path / "action.json").open("r", encoding="utf-8") as file:
        action = json.load(file)
    states = [np.load(path / manifest["states"][name]) for name in ("initial", "loaded", "residual")]
    return manifest, action, np.load(path / "valid_heightmap_mask.npy").astype(bool), states


def load_genesis_states(path: Path) -> tuple[dict, np.ndarray, list[np.ndarray]]:
    with (path / "manifest.json").open("r", encoding="utf-8") as file:
        manifest = json.load(file)
    states = [np.load(path / f"{name}_heightmap_m.npy") for name in ("initial", "loaded", "residual")]
    return manifest, np.load(path / "valid_heightmap_mask.npy").astype(bool), states


def main() -> None:
    args = parse_args()
    chrono_manifest, action, chrono_mask, chrono_states = load_chrono_states(args.chrono_episode)
    genesis_manifest, genesis_mask, genesis_states = load_genesis_states(args.genesis_run)
    if chrono_states[0].shape != genesis_states[0].shape:
        raise ValueError("Chrono and Genesis heightmap shapes differ")
    valid = chrono_mask & genesis_mask
    chrono_delta = [state - chrono_states[0] for state in chrono_states]
    genesis_delta = [state - genesis_states[0] for state in genesis_states]
    all_values = np.concatenate([field[valid] for field in (*chrono_delta, *genesis_delta)])
    limit_mm = max(float(np.max(np.abs(all_values))) * 1000.0, 1.0)
    heightmap = chrono_manifest["heightmap"]
    x0, y0 = heightmap["origin_xy_m"]
    spacing = float(heightmap["spacing_m"])
    rows, cols = heightmap["shape"]
    extent = (x0 - 0.5 * spacing, x0 + (cols - 0.5) * spacing, y0 - 0.5 * spacing, y0 + (rows - 0.5) * spacing)

    figure, axes = plt.subplots(2, 3, figsize=(13, 8), layout="constrained", sharex=True, sharey=True)
    rows_data = (("Chrono SCM", chrono_delta), ("Genesis MPM (pre-fit)", genesis_delta))
    state_names = ("Initial", "Loaded / during", "Residual / after")
    image = None
    for row_index, (row_name, fields) in enumerate(rows_data):
        for column_index, (state_name, field) in enumerate(zip(state_names, fields, strict=True)):
            axis = axes[row_index, column_index]
            plotted = np.ma.array(field * 1000.0, mask=~valid)
            image = axis.imshow(
                plotted,
                extent=extent,
                origin="lower",
                cmap="RdBu",
                vmin=-limit_mm,
                vmax=limit_mm,
                interpolation="nearest",
            )
            axis.add_patch(Circle(action["center_xy_m"], action["radius_m"], fill=False, color="black", linewidth=1.2))
            axis.set_aspect("equal")
            axis.set_title(f"{row_name}: {state_name}")
            if column_index == 0:
                axis.set_ylabel("bed y (m)")
            if row_index == 1:
                axis.set_xlabel("bed x (m)")
    figure.colorbar(image, ax=axes, shrink=0.82, label="surface elevation change from initial (mm)")
    figure.suptitle(
        f"{chrono_manifest['episode_id']} smoke bridge — cylinder {action['mass_kg']:.1f} kg, "
        f"valid cells {int(np.count_nonzero(valid))}/{valid.size}",
        fontsize=13,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == "__main__":
    main()
