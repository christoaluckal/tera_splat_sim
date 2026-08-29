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
    parser.add_argument(
        "--scm-grid-spacing-m",
        type=float,
        default=None,
        help="Override SCM grid spacing for an oracle-validation episode.",
    )
    parser.add_argument(
        "--scm-pit-size-m",
        type=float,
        nargs=2,
        default=None,
        metavar=("SIZE_X_M", "SIZE_Y_M"),
        help="Override SCM patch size for a fast local oracle screen.",
    )
    parser.add_argument(
        "--settle-time-s",
        type=float,
        default=None,
        help="Deprecated alias for --max-loading-time-s.",
    )
    parser.add_argument(
        "--max-loading-time-s",
        type=float,
        default=None,
        help="Maximum loading duration before a non-converged episode is rejected as an oracle.",
    )
    parser.add_argument(
        "--loading-linear-speed-threshold-mps",
        type=float,
        default=0.006,
        help="Required maximum cylinder linear speed for loading convergence (default: 6 mm/s).",
    )
    parser.add_argument(
        "--loading-angular-speed-threshold-radps",
        type=float,
        default=None,
        help="Required maximum cylinder angular speed for loading convergence (defaults to world config).",
    )
    parser.add_argument(
        "--loading-hold-time-s",
        type=float,
        default=0.10,
        help="Continuous time below both speed thresholds required for loading acceptance (default: 0.10 s).",
    )
    parser.add_argument(
        "--min-loading-time-s",
        type=float,
        default=0.25,
        help="Earliest time at which loading convergence can begin.",
    )
    parser.add_argument("--residual-settle-s", type=float, default=0.5, help="Recorded fixed post-removal recovery duration.")
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
    parser.add_argument(
        "--vertical-guide",
        action="store_true",
        help="Constrain the cylinder to vertical translation while it settles under gravity.",
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
        scm_grid_spacing_m=args.scm_grid_spacing_m,
        scm_pit_size_m=(tuple(float(value) for value in args.scm_pit_size_m) if args.scm_pit_size_m else None),
        settle_time_s=args.settle_time_s,
        max_loading_time_s=args.max_loading_time_s,
        loading_linear_speed_threshold_mps=args.loading_linear_speed_threshold_mps,
        loading_angular_speed_threshold_radps=args.loading_angular_speed_threshold_radps,
        loading_hold_time_s=args.loading_hold_time_s,
        min_loading_time_s=args.min_loading_time_s,
        residual_settle_s=args.residual_settle_s,
        capture_interval_s=args.capture_interval_s,
        vertical_guide=args.vertical_guide,
    )
    print(result["output_dir"])
    print(f"loaded_termination_reason: {result['loaded_termination_reason']}")
    print(f"loaded_convergence_accepted: {result['loaded_convergence_accepted']}")


if __name__ == "__main__":
    main()
