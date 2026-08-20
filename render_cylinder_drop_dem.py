#!/usr/bin/env python3
"""Render a captured Chrono cylinder drop beside its measured SCM DEM change."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chrono-episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--frames-per-snapshot", type=int, default=3)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0 or args.frames_per_snapshot <= 0:
        raise ValueError("fps and frames-per-snapshot must be positive")
    episode = args.chrono_episode.resolve()
    with (episode / "manifest.yaml").open("r", encoding="utf-8") as file:
        manifest = yaml.safe_load(file)
    with (episode / "action.json").open("r", encoding="utf-8") as file:
        action = json.load(file)
    snapshot_dir = episode / "terrain_snapshots"
    with (snapshot_dir / "manifest.json").open("r", encoding="utf-8") as file:
        snapshots = json.load(file)["records"]
    if not snapshots:
        raise ValueError("No terrain snapshots to render")
    initial = np.load(snapshot_dir / snapshots[0]["heightmap"])
    mask = np.load(episode / "valid_heightmap_mask.npy").astype(bool)
    fields = [np.load(snapshot_dir / record["heightmap"]) for record in snapshots]
    depression_max_mm = max(float(np.max((initial - field)[mask] * 1000.0)) for field in fields)
    depression_max_mm = max(depression_max_mm, 1e-3)
    spec = manifest["heightmap"]
    spacing = float(spec["spacing_m"])
    x0, y0 = (float(value) for value in spec["origin_xy_m"])
    rows, cols = (int(value) for value in spec["shape"])
    xs = x0 + spacing * np.arange(cols)
    center_row = int(np.argmin(np.abs(y0 + spacing * np.arange(rows) - float(action["center_xy_m"][1]))))
    extent = (x0 - 0.5 * spacing, x0 + (cols - 0.5) * spacing, y0 - 0.5 * spacing, y0 + (rows - 0.5) * spacing)

    figure, (scene_ax, dem_ax) = plt.subplots(1, 2, figsize=(12, 5.6), layout="constrained")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(args.output, fps=args.fps, codec="libx264", quality=8) as writer:
        for record, field in zip(snapshots, fields, strict=True):
            for _ in range(args.frames_per_snapshot):
                scene_ax.clear()
                dem_ax.clear()
                scene_ax.fill_between(xs, -0.08, field[center_row], color="#b68755", alpha=0.95)
                scene_ax.plot(xs, field[center_row], color="#3b2818", linewidth=1.4)
                if record["body_z_m"] is not None:
                    bottom_z = float(record["body_z_m"]) - 0.5 * float(action["height_m"])
                    scene_ax.add_patch(
                        Rectangle(
                            (float(record["body_x_m"]) - float(action["radius_m"]), bottom_z),
                            2.0 * float(action["radius_m"]),
                            float(action["height_m"]),
                            facecolor="#50677e",
                            edgecolor="#1b2730",
                            linewidth=1.2,
                        )
                    )
                scene_ax.axhline(0.0, color="#808080", linestyle="--", linewidth=0.8)
                scene_ax.set(xlim=(-0.18, 0.18), ylim=(-0.08, 0.10), xlabel="bed x (m)", ylabel="bed z (m)")
                scene_ax.set_aspect("equal", adjustable="box")
                scene_ax.set_title(f"Chrono SCM cylinder — {record['phase']}")

                deformation = np.ma.array((initial - field) * 1000.0, mask=~mask)
                image = dem_ax.imshow(
                    deformation,
                    extent=extent,
                    origin="lower",
                    cmap="viridis",
                    vmin=0.0,
                    vmax=depression_max_mm,
                    interpolation="nearest",
                )
                dem_ax.add_patch(Circle(action["center_xy_m"], action["radius_m"], fill=False, color="white", linewidth=1.1))
                dem_ax.set(xlabel="bed x (m)", ylabel="bed y (m)", aspect="equal", title="Measured SCM deformation from initial")
                figure.suptitle(
                    f"{manifest['episode_id']} — {action['mass_kg']:.1f} kg, "
                    f"{action['start_clearance_m'] * 1000:.0f} mm clearance, t = {record['time_s']:.3f} s"
                )
                if not hasattr(main, "colorbar"):
                    main.colorbar = figure.colorbar(image, ax=dem_ax, shrink=0.82, label="downward deformation (mm)")
                figure.canvas.draw()
                frame = np.asarray(figure.canvas.buffer_rgba())[:, :, :3]
                writer.append_data(frame)
    plt.close(figure)
    print(args.output)


if __name__ == "__main__":
    main()
