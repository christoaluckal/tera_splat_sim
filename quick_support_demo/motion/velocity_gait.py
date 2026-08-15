from __future__ import annotations

from dataclasses import dataclass

import numpy as np


LEGS = ("FR", "FL", "RR", "RL")
DIAGONAL_PHASE = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}


@dataclass(frozen=True)
class VelocityCommand:
    """Planar body velocity expressed in the robot body frame."""

    vx_mps: float = 0.0
    vy_mps: float = 0.0
    wz_radps: float = 0.0


@dataclass(frozen=True)
class GaitState:
    joint_positions: dict[str, float]
    foot_positions_body: dict[str, np.ndarray]
    stance: dict[str, bool]
    phase: float


class TrotGait:
    """Convert a planar velocity command into open-loop Go1 leg targets."""

    def __init__(
        self,
        body_com_height_m: float,
        foot_height_m: float,
        frequency_hz: float = 1.6,
        duty_factor: float = 0.58,
        step_height_m: float = 0.055,
    ) -> None:
        if frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be positive")
        if not 0.5 <= duty_factor < 1.0:
            raise ValueError("duty_factor must be in [0.5, 1.0)")
        if step_height_m < 0.0:
            raise ValueError("step_height_m must be non-negative")
        self.frequency_hz = frequency_hz
        self.duty_factor = duty_factor
        self.step_height_m = step_height_m
        self.hip_x = 0.1881
        self.hip_y = 0.04675
        self.hip_lateral_offset = 0.08
        self.link_length = 0.213
        self.nominal_foot_z = -body_com_height_m + 0.5 * foot_height_m

    def nominal_foot(self, leg: str) -> np.ndarray:
        front = 1.0 if leg.startswith("F") else -1.0
        left = 1.0 if leg.endswith("L") else -1.0
        return np.array(
            [
                front * self.hip_x,
                left * (self.hip_y + self.hip_lateral_offset),
                self.nominal_foot_z,
            ],
            dtype=float,
        )

    def sample(self, time_s: float, command: VelocityCommand) -> GaitState:
        speed = np.hypot(command.vx_mps, command.vy_mps)
        moving = speed > 1.0e-5 or abs(command.wz_radps) > 1.0e-5
        cycle_phase = (max(time_s, 0.0) * self.frequency_hz) % 1.0
        feet: dict[str, np.ndarray] = {}
        stance: dict[str, bool] = {}
        joints: dict[str, float] = {}

        for leg in LEGS:
            nominal = self.nominal_foot(leg)
            foot = nominal.copy()
            if moving:
                phase = (cycle_phase + DIAGONAL_PHASE[leg]) % 1.0
                contact_velocity = np.array(
                    [
                        command.vx_mps - command.wz_radps * nominal[1],
                        command.vy_mps + command.wz_radps * nominal[0],
                    ]
                )
                stride = contact_velocity * (self.duty_factor / self.frequency_hz)
                if phase < self.duty_factor:
                    progress = phase / self.duty_factor
                    foot[:2] += (0.5 - progress) * stride
                    stance[leg] = True
                else:
                    progress = (phase - self.duty_factor) / (1.0 - self.duty_factor)
                    blend = 0.5 - 0.5 * np.cos(np.pi * progress)
                    foot[:2] += (-0.5 + blend) * stride
                    foot[2] += self.step_height_m * np.sin(np.pi * progress)
                    stance[leg] = False
            else:
                stance[leg] = True

            feet[leg] = foot
            hip, thigh, calf = self._inverse_kinematics(leg, foot)
            joints[f"{leg}_hip_joint"] = hip
            joints[f"{leg}_thigh_joint"] = thigh
            joints[f"{leg}_calf_joint"] = calf

        return GaitState(joints, feet, stance, cycle_phase)

    def state_from_feet(
        self,
        foot_positions_body: dict[str, np.ndarray],
        stance: dict[str, bool],
        phase: float,
    ) -> GaitState:
        joints = {}
        positions = {}
        for leg in LEGS:
            foot = np.asarray(foot_positions_body[leg], dtype=float).copy()
            positions[leg] = foot
            hip, thigh, calf = self._inverse_kinematics(leg, foot)
            joints[f"{leg}_hip_joint"] = hip
            joints[f"{leg}_thigh_joint"] = thigh
            joints[f"{leg}_calf_joint"] = calf
        return GaitState(joints, positions, dict(stance), phase)

    def _inverse_kinematics(self, leg: str, foot_body: np.ndarray) -> tuple[float, float, float]:
        front = 1.0 if leg.startswith("F") else -1.0
        left = 1.0 if leg.endswith("L") else -1.0
        target = foot_body - np.array([front * self.hip_x, left * self.hip_y, 0.0])
        lateral_offset = left * self.hip_lateral_offset

        radius_yz = float(np.hypot(target[1], target[2]))
        if radius_yz <= abs(lateral_offset):
            raise ValueError(f"Unreachable lateral target for {leg}: {foot_body}")
        phi = float(np.arctan2(target[2], target[1]))
        delta = float(np.arccos(np.clip(lateral_offset / radius_yz, -1.0, 1.0)))
        candidates = (phi + delta, phi - delta)
        hip = min(candidates, key=lambda angle: abs(np.arctan2(np.sin(angle), np.cos(angle))))
        hip = float(np.arctan2(np.sin(hip), np.cos(hip)))

        c, s = np.cos(hip), np.sin(hip)
        sagittal_x = float(target[0])
        sagittal_z = float(-s * target[1] + c * target[2])
        length = self.link_length
        cos_calf = (sagittal_x**2 + sagittal_z**2 - 2.0 * length**2) / (2.0 * length**2)
        if cos_calf < -1.0001 or cos_calf > 1.0001:
            raise ValueError(f"Unreachable sagittal target for {leg}: {foot_body}")
        calf = -float(np.arccos(np.clip(cos_calf, -1.0, 1.0)))
        alpha = float(np.arctan2(-sagittal_x, -sagittal_z))
        beta = float(np.arctan2(length * np.sin(calf), length + length * np.cos(calf)))
        thigh = alpha - beta
        return hip, thigh, calf

    def forward_kinematics(self, leg: str, joint_positions: dict[str, float]) -> np.ndarray:
        """Return a foot center in the body frame for validation and overlays."""
        front = 1.0 if leg.startswith("F") else -1.0
        left = 1.0 if leg.endswith("L") else -1.0
        hip = joint_positions[f"{leg}_hip_joint"]
        thigh = joint_positions[f"{leg}_thigh_joint"]
        calf = joint_positions[f"{leg}_calf_joint"]
        length = self.link_length
        sagittal = np.array(
            [
                -length * np.sin(thigh) - length * np.sin(thigh + calf),
                left * self.hip_lateral_offset,
                -length * np.cos(thigh) - length * np.cos(thigh + calf),
            ]
        )
        c, s = np.cos(hip), np.sin(hip)
        rotated = np.array(
            [
                sagittal[0],
                c * sagittal[1] - s * sagittal[2],
                s * sagittal[1] + c * sagittal[2],
            ]
        )
        return rotated + np.array([front * self.hip_x, left * self.hip_y, 0.0])
