from __future__ import annotations

import math

from .build_world import make_contact_material
from .chrono_import import import_chrono


chrono, _veh = import_chrono()


def foot_offsets(robot_cfg: dict) -> list[tuple[float, float, float]]:
    robot = robot_cfg["robot"]
    lx = 0.5 * float(robot["stance_length_m"])
    ly = 0.5 * float(robot["stance_width_m"])
    z = -float(robot["body_com_height_m"]) + 0.5 * float(robot["foot_height_m"])
    return [(lx, ly, z), (lx, -ly, z), (-lx, ly, z), (-lx, -ly, z)]


def build_support_proxy(
    system: object,
    robot_cfg: dict,
    base_xy: tuple[float, float],
    foot_collisions: bool = True,
    body_collision: bool = False,
    contact_friction: float = 0.9,
    collision_foot_indices: set[int] | None = None,
) -> object:
    robot = robot_cfg["robot"]
    mass = float(robot["mass_kg"]) + float(robot.get("payload_kg", 0.0))
    body_size = [float(v) for v in robot["body_size_m"]]
    foot_radius = float(robot["foot_radius_m"])
    foot_height = float(robot["foot_height_m"])
    foot_side = math.sqrt(math.pi * foot_radius * foot_radius)
    z0 = float(robot["body_com_height_m"]) + float(robot["start_clearance_m"])

    material = make_contact_material(contact_friction)
    body = chrono.ChBody()
    body.SetName(f"{robot['name']}_support_proxy")
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(
        mass * (body_size[1] ** 2 + body_size[2] ** 2) / 12.0,
        mass * (body_size[0] ** 2 + body_size[2] ** 2) / 12.0,
        mass * (body_size[0] ** 2 + body_size[1] ** 2) / 12.0,
    ))
    body.SetPos(chrono.ChVector3d(float(base_xy[0]), float(base_xy[1]), z0))
    body.SetRot(chrono.QUNIT)
    body.EnableCollision(True)

    if body_collision:
        body.AddCollisionShape(
            chrono.ChCollisionShapeBox(material, *body_size),
            chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
        )

    has_robot_visual = False
    if robot.get("visual_asset") == "go1_urdf":
        from quick_support_demo.robot_assets.go1 import ensure_go1_visual_cache

        _cache_path, obj_path = ensure_go1_visual_cache(robot_cfg)
        triangle_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(str(obj_path), True, False)
        mesh_shape = chrono.ChVisualShapeTriangleMesh()
        mesh_shape.SetMesh(triangle_mesh, False)
        mesh_shape.SetColor(chrono.ChColor(0.32, 0.34, 0.36))
        body.AddVisualShape(mesh_shape)
        has_robot_visual = True

    if not has_robot_visual:
        body_shape = chrono.ChVisualShapeBox(*body_size)
        body.AddVisualShape(body_shape, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))

    for index, (x, y, z) in enumerate(foot_offsets(robot_cfg)):
        frame = chrono.ChFramed(chrono.ChVector3d(x, y, z), chrono.QUNIT)
        if foot_collisions and (collision_foot_indices is None or index in collision_foot_indices):
            foot_shape = chrono.ChCollisionShapeBox(material, foot_side, foot_side, foot_height)
            body.AddCollisionShape(foot_shape, frame)
        if not has_robot_visual:
            visual = chrono.ChVisualShapeBox(foot_side, foot_side, foot_height)
            body.AddVisualShape(visual, frame)

    system.Add(body)
    return body


def world_foot_bottoms(body: object, robot_cfg: dict) -> list[tuple[float, float, float]]:
    rot = body.GetRot()
    pos = body.GetPos()
    foot_height = float(robot_cfg["robot"]["foot_height_m"])
    bottoms = []
    for x, y, z in foot_offsets(robot_cfg):
        world = pos + rot.Rotate(chrono.ChVector3d(x, y, z - 0.5 * foot_height))
        bottoms.append((float(world.x), float(world.y), float(world.z)))
    return bottoms
