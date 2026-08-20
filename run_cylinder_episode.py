from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from quick_support_demo.chrono_demo.cylinder_episode import CylinderAction, run_cylinder_episode
from quick_support_demo.config import PROJECT_ROOT


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a stationary Chrono SCM cylinder episode for sim-only validation.")
    parser.add_argument("--episode-id", default="A0_cal")
    parser.add_argument("--mass-kg", type=float, default=1.5)
    parser.add_argument("--xy", type=float, nargs=2, default=(0.0, 0.0), metavar=("X_M", "Y_M"))
    parser.add_argument("--smoke", action="store_true", help="Use the coarse SCM grid for plumbing only.")
    parser.add_argument("--timestep-s", type=float, default=None, help="Override the Chrono integration timestep.")
    parser.add_argument("--settle-time-s", type=float, default=None, help="Override the loaded-settle duration.")
    parser.add_argument("--residual-settle-s", type=float, default=0.5, help="Post-removal recovery duration.")
    parser.add_argument(
        "--start-clearance-m",
        type=float,
        default=0.02,
        help="Initial bottom-face clearance above the SCM surface.",
    )
    parser.add_argument(
        "--capture-interval-s",
        type=float,
        default=None,
        help="Optional SCM heightmap capture interval for time-resolved rendering.",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir or (
        PROJECT_ROOT
        / "validity_experiment"
        / "chrono_episodes"
        / f"{args.episode_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    result = run_cylinder_episode(
        output_dir=output_dir,
        action=CylinderAction(
            args.episode_id,
            args.mass_kg,
            (float(args.xy[0]), float(args.xy[1])),
            start_clearance_m=float(args.start_clearance_m),
        ),
        smoke=args.smoke,
        timestep_s=args.timestep_s,
        settle_time_s=args.settle_time_s,
        residual_settle_s=args.residual_settle_s,
        capture_interval_s=args.capture_interval_s,
    )
    print(result["output_dir"])
    print(f"loaded_termination_reason: {result['loaded_termination_reason']}")


if __name__ == "__main__":
    main()
