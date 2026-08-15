from __future__ import annotations

from quick_support_demo.chrono_demo.outcomes import SupportOutcome


def objective_cost(outcome: SupportOutcome, candidate_cfg: dict, planning_cfg: dict) -> float:
    weights = planning_cfg["weights"]
    return float(
        weights["view"] * float(candidate_cfg["view_cost"])
        + weights["path"] * float(candidate_cfg["path_cost"])
        + weights["sinkage"] * outcome.max_sinkage_m
        + weights["tilt"] * outcome.max_abs_tilt_rad
        + weights["uncertainty"] * float(candidate_cfg.get("uncertainty", 0.0))
    )

