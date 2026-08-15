from __future__ import annotations

from quick_support_demo.chrono_demo.outcomes import SupportOutcome

from .support_cost import objective_cost


def select_candidate(
    outcomes: dict[str, SupportOutcome],
    candidate_cfgs: dict,
    planning_cfg: dict,
) -> tuple[str, dict[str, float]]:
    costs = {
        name: objective_cost(outcome, candidate_cfgs[name], planning_cfg)
        for name, outcome in outcomes.items()
    }
    selected = min(costs, key=costs.get)
    return selected, costs

