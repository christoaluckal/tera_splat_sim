from __future__ import annotations

import time

import numpy as np

from .build_scm_pit import build_scm_pit, sample_heightmap
from .build_support_proxy import build_support_proxy, world_foot_bottoms
from .build_world import add_perimeter_floor, build_system
from .chrono_import import import_chrono
from .outcomes import SupportOutcome


chrono, _veh = import_chrono()


def _vec_length(vec: object) -> float:
    return float((vec.x * vec.x + vec.y * vec.y + vec.z * vec.z) ** 0.5)


def _roll_pitch(body: object) -> tuple[float, float]:
    angles = body.GetRot().GetCardanAnglesXYZ()
    return float(angles.x), float(angles.y)


def run_support_trial(
    world_cfg: dict,
    terrain_cfg: dict,
    robot_cfg: dict,
    candidate_name: str,
    candidate_cfg: dict,
    residual_settle_s: float = 0.5,
) -> SupportOutcome:
    system = build_system(world_cfg)
    add_perimeter_floor(system, world_cfg, terrain_cfg)
    terrain = build_scm_pit(system, terrain_cfg, visualization_mesh=False)
    initial_heightmap = sample_heightmap(terrain, terrain_cfg)

    base_xy = tuple(candidate_cfg["base_xy_m"])
    body = build_support_proxy(system, robot_cfg, base_xy)
    initial_com_z = float(body.GetPos().z)
    dt = float(world_cfg["world"]["timestep_s"])
    settle_time = float(world_cfg["world"]["settle_time_s"])
    lin_threshold = float(world_cfg["world"]["settle_velocity_mps"])
    ang_threshold = float(world_cfg["world"]["settle_ang_velocity_radps"])

    start = time.perf_counter()
    steps = int(settle_time / dt)
    settled_steps = 0
    for _ in range(steps):
        terrain.Synchronize(system.GetChTime())
        system.DoStepDynamics(dt)
        terrain.Advance(dt)
        lin = _vec_length(body.GetPosDt())
        ang = _vec_length(body.GetAngVelParent())
        if lin < lin_threshold and ang < ang_threshold and system.GetChTime() > 0.25:
            settled_steps += 1
            if settled_steps > int(0.10 / dt):
                break
        else:
            settled_steps = 0

    loaded_heightmap = sample_heightmap(terrain, terrain_cfg)
    foot_bottoms = world_foot_bottoms(body, robot_cfg)
    foot_sinkage = np.array([max(0.0, -z) for _x, _y, z in foot_bottoms], dtype=np.float32)
    roll, pitch = _roll_pitch(body)
    com_height_change = float(body.GetPos().z) - initial_com_z

    system.Remove(body)
    for _ in range(int(residual_settle_s / dt)):
        terrain.Synchronize(system.GetChTime())
        system.DoStepDynamics(dt)
        terrain.Advance(dt)
    residual_heightmap = sample_heightmap(terrain, terrain_cfg)

    return SupportOutcome(
        robot=str(robot_cfg["robot"]["name"]),
        candidate_pose=candidate_name,
        foot_sinkage_m=foot_sinkage,
        body_roll_rad=roll,
        body_pitch_rad=pitch,
        com_height_change_m=com_height_change,
        initial_heightmap_m=initial_heightmap,
        loaded_heightmap_m=loaded_heightmap,
        residual_heightmap_m=residual_heightmap,
        runtime_s=time.perf_counter() - start,
    )

