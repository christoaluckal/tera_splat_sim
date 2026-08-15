from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def latest_trial_root() -> Path:
    trials = PROJECT_ROOT / "quick_support_demo" / "outputs" / "trials"
    roots = sorted(p for p in trials.iterdir() if p.is_dir())
    if not roots:
        raise FileNotFoundError(f"No trial outputs found under {trials}")
    return roots[-1]


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_case(root: Path, robot: str, candidate: str) -> tuple[dict, np.ndarray, np.ndarray]:
    outcome = load_json(root / robot / candidate / "outcome.json")
    initial = np.load(root / robot / candidate / "initial_heightmap_m.npy")
    loaded = np.load(root / robot / candidate / "loaded_heightmap_m.npy")
    return outcome, initial, loaded


def add_panel(ax, title: str, sinkage: np.ndarray, outcome: dict, selected: str, note: str) -> None:
    im = ax.imshow(sinkage * 1000.0, vmin=0.0, vmax=120.0, cmap="magma", origin="lower")
    ax.set_title(title, fontsize=18, pad=8)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_linewidth(2)
        spine.set_edgecolor("#303030")

    foot_sinkage_mm = [1000.0 * float(v) for v in outcome["foot_sinkage_m"]]
    label = (
        f"selected: {selected}\n"
        f"max sinkage: {1000.0 * outcome['max_sinkage_m']:.1f} mm\n"
        f"tilt: {np.degrees(outcome['max_abs_tilt_rad']):.2f} deg\n"
        f"{note}"
    )
    ax.text(
        0.03,
        0.97,
        label,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=12,
        color="white",
        bbox={"facecolor": "black", "alpha": 0.72, "pad": 8, "edgecolor": "none"},
    )
    ax.text(
        0.03,
        0.05,
        "foot sinkage mm: " + ", ".join(f"{v:.1f}" for v in foot_sinkage_mm),
        transform=ax.transAxes,
        va="bottom",
        ha="left",
        fontsize=10,
        color="white",
        bbox={"facecolor": "black", "alpha": 0.55, "pad": 5, "edgecolor": "none"},
    )
    return im


def make_video(trial_root: Path, output_path: Path, fps: int, seconds: float) -> None:
    go1_summary = load_json(trial_root / "go1" / "summary.json")
    spot_summary = load_json(trial_root / "spot" / "summary.json")

    go1_outcome, go1_initial, go1_loaded = load_case(trial_root, "go1", "sand")
    spot_counter, spot_initial, spot_loaded = load_case(trial_root, "spot", "sand")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    n_frames = int(fps * seconds)
    with imageio.get_writer(output_path, fps=fps, codec="libx264", quality=8) as writer:
        for frame_idx in range(n_frames):
            t = frame_idx / max(1, n_frames - 1)
            ease = 3.0 * t * t - 2.0 * t * t * t
            go1_height = (1.0 - ease) * go1_initial + ease * go1_loaded
            spot_height = (1.0 - ease) * spot_initial + ease * spot_loaded
            go1_sinkage = np.maximum(0.0, go1_initial - go1_height)
            spot_sinkage = np.maximum(0.0, spot_initial - spot_height)

            fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.2), dpi=100)
            fig.patch.set_facecolor("#f2f2f0")
            im = add_panel(
                axes[0],
                "Go1: selected sand pose",
                go1_sinkage,
                go1_outcome,
                go1_summary["selected_candidate"],
                "SCM rollout executed",
            )
            add_panel(
                axes[1],
                "Spot: sand rejected, rigid selected",
                spot_sinkage,
                spot_counter,
                spot_summary["selected_candidate"],
                "sand counterfactual shown",
            )
            cbar = fig.colorbar(im, ax=axes, location="bottom", fraction=0.055, pad=0.08)
            cbar.set_label("terrain sinkage (mm)")
            fig.suptitle(
                "Same terrain, different embodiment, different physically viable plan",
                fontsize=20,
                y=0.97,
            )
            fig.canvas.draw()
            frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
            writer.append_data(frame)
            plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create a 2D preview video from support trial outputs.")
    parser.add_argument("--trial-root", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--seconds", type=float, default=6.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trial_root = args.trial_root or latest_trial_root()
    output = args.output or PROJECT_ROOT / "quick_support_demo" / "outputs" / "videos" / f"{trial_root.name}_preview.mp4"
    make_video(trial_root.resolve(), output.resolve(), args.fps, args.seconds)
    print(output.resolve())


if __name__ == "__main__":
    main()

