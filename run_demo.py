from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from quick_support_demo.chrono_demo.outcomes import save_heightmaps
from quick_support_demo.chrono_demo.run_support_trial import run_support_trial
from quick_support_demo.config import PROJECT_ROOT, load_demo_config
from quick_support_demo.planning.select_pose import select_candidate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the PyChrono quick support proxy demo.")
    parser.add_argument("--robot", choices=["go1", "spot", "all"], default="all")
    parser.add_argument("--candidate", choices=["sand", "rigid", "all"], default="all")
    parser.add_argument("--smoke", action="store_true", help="Use faster coarse settings for API/debug validation.")
    parser.add_argument("--output-root", default="quick_support_demo/outputs/trials")
    return parser.parse_args()


def apply_smoke_overrides(cfg: dict) -> None:
    cfg["world"]["world"]["timestep_s"] = 0.001
    cfg["world"]["world"]["settle_time_s"] = 0.6
    cfg["terrain"]["pit"]["grid_spacing_m"] = 0.04


def run_robot(robot_name: str, cfg: dict, output_root: Path, candidate_filter: str) -> dict:
    robot_cfg = cfg["robots"][robot_name]
    candidates = cfg["candidates"]["candidates"]
    if candidate_filter != "all":
        candidates = {candidate_filter: candidates[candidate_filter]}
    planning = cfg["candidates"]["planning"]
    outcomes = {}

    for candidate_name, candidate_cfg in candidates.items():
        outcome = run_support_trial(
            cfg["world"],
            cfg["terrain"],
            robot_cfg,
            candidate_name,
            candidate_cfg,
        )
        outcomes[candidate_name] = outcome

    selected, costs = select_candidate(outcomes, candidates, planning)
    robot_dir = output_root / robot_name
    robot_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        "robot": robot_name,
        "selected_candidate": selected,
        "costs": costs,
        "outcomes": {},
    }
    for candidate_name, outcome in outcomes.items():
        outcome.selected_candidate = selected
        outcome.total_cost = costs[candidate_name]
        candidate_dir = robot_dir / candidate_name
        candidate_dir.mkdir(parents=True, exist_ok=True)
        save_heightmaps(outcome, candidate_dir)
        data = outcome.to_json_dict()
        summary["outcomes"][candidate_name] = data
        with (candidate_dir / "outcome.json").open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    with (robot_dir / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    return summary


def main() -> None:
    args = parse_args()
    cfg = load_demo_config()
    if args.smoke:
        apply_smoke_overrides(cfg)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = PROJECT_ROOT / args.output_root / stamp
    robots = ["go1", "spot"] if args.robot == "all" else [args.robot]

    summaries = [run_robot(robot, cfg, output_root, args.candidate) for robot in robots]
    print(json.dumps({"output_root": str(output_root), "summaries": summaries}, indent=2))


if __name__ == "__main__":
    main()
