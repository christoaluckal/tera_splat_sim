from __future__ import annotations

from typing import Iterable

from .chrono_import import import_chrono


chrono, _veh = import_chrono()


def make_contact_material(friction: float) -> object:
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(float(friction))
    material.SetYoungModulus(1.0e6)
    material.SetRestitution(0.05)
    return material


def build_system(world_cfg: dict) -> object:
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(*world_cfg["world"]["gravity_mps2"]))
    return system


def add_box(
    system: object,
    name: str,
    size_xyz: Iterable[float],
    center_xyz: Iterable[float],
    material: object,
    density: float,
    fixed: bool = True,
    color: tuple[float, float, float] | None = None,
    collide: bool = True,
) -> object:
    sx, sy, sz = [float(v) for v in size_xyz]
    body = chrono.ChBodyEasyBox(sx, sy, sz, float(density), True, collide, material)
    body.SetName(name)
    body.SetPos(chrono.ChVector3d(*[float(v) for v in center_xyz]))
    body.SetFixed(fixed)
    if color is not None:
        body.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(*color))
    system.Add(body)
    return body


def add_perimeter_floor(system: object, world_cfg: dict, terrain_cfg: dict) -> list[object]:
    floor_size_x, floor_size_y = world_cfg["world"]["floor_size_m"]
    pit_size_x, pit_size_y = terrain_cfg["pit"]["size_m"]
    thickness = float(world_cfg["floor"]["thickness_m"])
    density = float(world_cfg["floor"]["density_kgpm3"])
    z = -0.5 * thickness
    material = make_contact_material(float(world_cfg["floor"]["friction"]))

    x_margin = 0.5 * (floor_size_x - pit_size_x)
    y_margin = 0.5 * (floor_size_y - pit_size_y)

    return [
        add_box(system, "floor_left", (x_margin, floor_size_y, thickness), (-(pit_size_x + x_margin) / 2, 0, z), material, density, color=(0.45, 0.46, 0.46)),
        add_box(system, "floor_right", (x_margin, floor_size_y, thickness), ((pit_size_x + x_margin) / 2, 0, z), material, density, color=(0.45, 0.46, 0.46)),
        add_box(system, "floor_near", (pit_size_x, y_margin, thickness), (0, -(pit_size_y + y_margin) / 2, z), material, density, color=(0.45, 0.46, 0.46)),
        add_box(system, "floor_far", (pit_size_x, y_margin, thickness), (0, (pit_size_y + y_margin) / 2, z), material, density, color=(0.45, 0.46, 0.46)),
    ]
