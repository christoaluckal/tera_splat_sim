from __future__ import annotations

from dataclasses import dataclass

from .velocity_gait import VelocityCommand


@dataclass(frozen=True)
class ManeuverState:
    command: VelocityCommand
    phase: str
    completed: bool


class ForwardTurnForward:
    """Time-parameterized forward, in-place turn, forward command sequence."""

    def __init__(
        self,
        forward_speed_mps: float,
        first_distance_m: float,
        turn_angle_rad: float,
        turn_rate_radps: float,
        second_distance_m: float,
    ) -> None:
        if forward_speed_mps <= 0.0:
            raise ValueError("forward_speed_mps must be positive")
        if first_distance_m < 0.0 or second_distance_m < 0.0:
            raise ValueError("forward distances must be nonnegative")
        if turn_rate_radps <= 0.0:
            raise ValueError("turn_rate_radps must be positive")

        self.forward_speed_mps = forward_speed_mps
        self.first_distance_m = first_distance_m
        self.turn_angle_rad = turn_angle_rad
        self.turn_rate_radps = turn_rate_radps
        self.second_distance_m = second_distance_m
        self.first_duration_s = first_distance_m / forward_speed_mps
        self.turn_duration_s = abs(turn_angle_rad) / turn_rate_radps
        self.second_duration_s = second_distance_m / forward_speed_mps
        self.total_duration_s = (
            self.first_duration_s + self.turn_duration_s + self.second_duration_s
        )

    def sample(self, elapsed_s: float) -> ManeuverState:
        elapsed = max(float(elapsed_s), 0.0)
        if elapsed < self.first_duration_s:
            return ManeuverState(
                VelocityCommand(vx_mps=self.forward_speed_mps),
                "forward 1",
                False,
            )

        elapsed -= self.first_duration_s
        if elapsed < self.turn_duration_s:
            turn_sign = 1.0 if self.turn_angle_rad >= 0.0 else -1.0
            return ManeuverState(
                VelocityCommand(wz_radps=turn_sign * self.turn_rate_radps),
                "turn left" if turn_sign > 0.0 else "turn right",
                False,
            )

        elapsed -= self.turn_duration_s
        if elapsed < self.second_duration_s:
            return ManeuverState(
                VelocityCommand(vx_mps=self.forward_speed_mps),
                "forward 2",
                False,
            )

        return ManeuverState(VelocityCommand(), "complete", True)
