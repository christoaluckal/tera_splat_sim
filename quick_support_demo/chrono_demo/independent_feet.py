from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from quick_support_demo.motion import GaitState

from .build_world import make_contact_material
from .chrono_import import import_chrono


chrono, _veh = import_chrono()
LEGS = ("FR", "FL", "RR", "RL")


@dataclass
class IndependentFeet:
    bodies: dict[str, object]
    stance: dict[str, bool]
    previous_xy: dict[str, np.ndarray]
    foot_side_m: float
    foot_height_m: float


def _set_foot_mass(body: object, mass_kg: float, side_m: float, height_m: float) -> None:
    body.SetMass(mass_kg)
    body.SetInertiaXX(
        chrono.ChVector3d(
            mass_kg * (side_m**2 + height_m**2) / 12.0,
            mass_kg * (side_m**2 + height_m**2) / 12.0,
            mass_kg * (2.0 * side_m**2) / 12.0,
        )
    )


def build_independent_feet(
    system: object,
    robot_cfg: dict,
    trunk_position: object,
    trunk_yaw_rad: float,
    gait_state: GaitState,
    total_mass_kg: float,
) -> IndependentFeet:
    robot = robot_cfg["robot"]
    foot_radius = float(robot["foot_radius_m"])
    foot_side = float(np.sqrt(np.pi * foot_radius**2))
    foot_height = float(robot["foot_height_m"])
    material = make_contact_material(0.9)
    yaw = chrono.QuatFromAngleZ(trunk_yaw_rad)
    bodies = {}
    previous_xy = {}
    stance_count = max(sum(gait_state.stance.values()), 1)

    for leg in LEGS:
        offset = gait_state.foot_positions_body[leg]
        world = trunk_position + yaw.Rotate(chrono.ChVector3d(*offset))
        body = chrono.ChBody()
        body.SetName(f"{leg}_independent_contact_foot")
        _set_foot_mass(body, total_mass_kg / stance_count, foot_side, foot_height)
        body.SetPos(world)
        body.SetRot(yaw)
        body.AddCollisionShape(
            chrono.ChCollisionShapeBox(material, foot_side, foot_side, foot_height),
            chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
        )
        body.SetFixed(not gait_state.stance[leg])
        body.EnableCollision(gait_state.stance[leg])
        system.Add(body)
        bodies[leg] = body
        previous_xy[leg] = np.array([world.x, world.y], dtype=float)

    return IndependentFeet(
        bodies=bodies,
        stance=dict(gait_state.stance),
        previous_xy=previous_xy,
        foot_side_m=foot_side,
        foot_height_m=foot_height,
    )


def update_independent_feet(
    feet: IndependentFeet,
    gait_state: GaitState,
    trunk_position: object,
    trunk_yaw_rad: float,
    total_mass_kg: float,
    dt: float,
) -> None:
    yaw = chrono.QuatFromAngleZ(trunk_yaw_rad)
    stance_count = max(sum(gait_state.stance.values()), 1)
    stance_mass = total_mass_kg / stance_count

    for leg, body in feet.bodies.items():
        offset = gait_state.foot_positions_body[leg]
        desired = trunk_position + yaw.Rotate(chrono.ChVector3d(*offset))
        previous_xy = feet.previous_xy[leg]
        planar_velocity = np.array([desired.x, desired.y]) - previous_xy
        planar_velocity /= dt
        entering_stance = gait_state.stance[leg] and not feet.stance[leg]

        if gait_state.stance[leg]:
            body.SetFixed(False)
            if entering_stance:
                body.SetPos(desired)
                body.SetLinVel(chrono.ChVector3d(planar_velocity[0], planar_velocity[1], 0.0))
            else:
                current = body.GetPos()
                vertical_velocity = body.GetPosDt().z
                body.SetPos(chrono.ChVector3d(desired.x, desired.y, current.z))
                body.SetLinVel(
                    chrono.ChVector3d(planar_velocity[0], planar_velocity[1], vertical_velocity)
                )
            _set_foot_mass(body, stance_mass, feet.foot_side_m, feet.foot_height_m)
            body.SetRot(yaw)
            body.EnableCollision(True)
        else:
            body.SetFixed(True)
            body.EnableCollision(False)
            body.SetPos(desired)
            body.SetRot(yaw)
            body.SetLinVel(chrono.VNULL)
            body.SetAngVelParent(chrono.VNULL)

        feet.previous_xy[leg] = np.array([desired.x, desired.y], dtype=float)
        feet.stance[leg] = gait_state.stance[leg]


def contact_adjusted_gait_state(
    gait,
    desired_state: GaitState,
    feet: IndependentFeet,
    trunk_position: object,
    trunk_yaw_rad: float,
) -> GaitState:
    inverse_yaw = chrono.QuatFromAngleZ(-trunk_yaw_rad)
    positions = {}
    for leg in LEGS:
        if desired_state.stance[leg]:
            relative = inverse_yaw.Rotate(feet.bodies[leg].GetPos() - trunk_position)
            positions[leg] = np.array([relative.x, relative.y, relative.z], dtype=float)
        else:
            positions[leg] = desired_state.foot_positions_body[leg]
    return gait.state_from_feet(positions, desired_state.stance, desired_state.phase)
