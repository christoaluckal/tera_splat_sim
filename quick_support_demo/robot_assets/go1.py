from __future__ import annotations

from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from quick_support_demo.config import PROJECT_ROOT


ASSET_ROOT = PROJECT_ROOT / "quick_support_demo" / "assets" / "go1"
URDF_PATH = ASSET_ROOT / "urdf" / "go1.urdf"
CACHE_DIR = ASSET_ROOT / "render_cache"
CACHE_NPZ = CACHE_DIR / "go1_standing.npz"
CACHE_OBJ = CACHE_DIR / "go1_standing.obj"

NOMINAL_JOINTS = {
    **{f"F{side}_hip_joint": 0.0 for side in ("R", "L")},
    **{f"F{side}_thigh_joint": 0.9 for side in ("R", "L")},
    **{f"F{side}_calf_joint": -1.8 for side in ("R", "L")},
    **{f"R{side}_hip_joint": 0.0 for side in ("R", "L")},
    **{f"R{side}_thigh_joint": 0.9 for side in ("R", "L")},
    **{f"R{side}_calf_joint": -1.8 for side in ("R", "L")},
}

TARGET_FACE_COUNTS = {
    "trunk.stl": 600,
    "hip.stl": 400,
    "thigh.stl": 300,
    "thigh_mirror.stl": 300,
    "calf.stl": 300,
}


def _numbers(value: str | None, default: tuple[float, ...]) -> np.ndarray:
    if not value:
        return np.asarray(default, dtype=float)
    return np.asarray([float(part) for part in value.split()], dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin_matrix(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, :3] = _rpy_matrix(_numbers(element.get("rpy"), (0.0, 0.0, 0.0)))
    transform[:3, 3] = _numbers(element.get("xyz"), (0.0, 0.0, 0.0))
    return transform


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm == 0.0 or angle == 0.0:
        return np.eye(4)
    x, y, z = axis / norm
    c, s = np.cos(angle), np.sin(angle)
    one_c = 1.0 - c
    rotation = np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ]
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform


def _link_transforms(root: ET.Element, joint_positions: dict[str, float] | None = None) -> dict[str, np.ndarray]:
    children: dict[str, list[ET.Element]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        if parent is not None:
            children.setdefault(parent.get("link", ""), []).append(joint)

    transforms = {"trunk": np.eye(4)}
    pending = ["trunk"]
    while pending:
        parent_name = pending.pop()
        for joint in children.get(parent_name, []):
            child = joint.find("child")
            if child is None:
                continue
            child_name = child.get("link", "")
            axis_element = joint.find("axis")
            axis = _numbers(axis_element.get("xyz") if axis_element is not None else None, (1.0, 0.0, 0.0))
            joint_name = joint.get("name", "")
            angle = (joint_positions or NOMINAL_JOINTS).get(joint_name, 0.0)
            transforms[child_name] = transforms[parent_name] @ _origin_matrix(joint.find("origin")) @ _axis_rotation(axis, angle)
            pending.append(child_name)
    return transforms


def _part_color(link_name: str) -> np.ndarray:
    if link_name == "trunk":
        return np.array([82, 88, 93, 255], dtype=np.uint8)
    if link_name.endswith("_hip"):
        return np.array([210, 119, 48, 255], dtype=np.uint8)
    if link_name.endswith("_thigh"):
        return np.array([94, 99, 102, 255], dtype=np.uint8)
    return np.array([54, 58, 61, 255], dtype=np.uint8)


def _load_source_mesh(mesh_path: Path, target_faces: int):
    import trimesh

    geometry = trimesh.load_scene(mesh_path).to_geometry()
    geometry.merge_vertices()
    geometry.remove_unreferenced_vertices()
    geometry = geometry.convex_hull
    if len(geometry.faces) > target_faces:
        geometry = geometry.simplify_quadric_decimation(face_count=target_faces, aggression=7)
    geometry.remove_unreferenced_vertices()
    return geometry


def _mesh_visual_transform(link_name: str, visual: ET.Element) -> np.ndarray:
    # MuJoCo Menagerie's clean STL export bakes different hip orientations than
    # Unitree's Collada files. These rotations match its Go1 MJCF definition.
    if link_name == "RR_hip":
        transform = np.eye(4)
        transform[:3, :3] = _rpy_matrix(np.array([0.0, 0.0, np.pi]))
        return transform
    if link_name == "RL_hip":
        transform = np.eye(4)
        transform[:3, :3] = _rpy_matrix(np.array([0.0, np.pi, 0.0]))
        return transform
    if link_name.endswith("_hip"):
        return np.eye(4)
    return _origin_matrix(visual.find("origin"))


def ensure_go1_visual_cache(robot_cfg: dict, force: bool = False) -> tuple[Path, Path]:
    if not URDF_PATH.exists():
        raise FileNotFoundError(f"Go1 URDF is missing: {URDF_PATH}")
    if CACHE_NPZ.exists() and CACHE_OBJ.exists() and not force:
        return CACHE_NPZ, CACHE_OBJ

    import trimesh

    root = ET.parse(URDF_PATH).getroot()
    transforms = _link_transforms(root)
    foot_z = np.mean([transforms[f"{leg}_foot"][2, 3] for leg in ("FR", "FL", "RR", "RL")])
    robot = robot_cfg["robot"]
    proxy_foot_center_z = -float(robot["body_com_height_m"]) + 0.5 * float(robot["foot_height_m"])
    root_shift = np.eye(4)
    root_shift[2, 3] = proxy_foot_center_z - foot_z

    source_meshes = {}
    all_vertices = []
    all_faces = []
    all_colors = []
    vertex_offset = 0

    for link in root.findall("link"):
        link_name = link.get("name", "")
        if link_name not in transforms:
            continue
        for visual in link.findall("visual"):
            mesh_element = visual.find("geometry/mesh")
            if mesh_element is None:
                continue
            urdf_mesh_name = Path(mesh_element.get("filename", "")).name
            mesh_name = f"{Path(urdf_mesh_name).stem}.stl"
            if mesh_name not in TARGET_FACE_COUNTS:
                continue
            if mesh_name not in source_meshes:
                source_meshes[mesh_name] = _load_source_mesh(
                    ASSET_ROOT / "simplified_meshes" / mesh_name,
                    TARGET_FACE_COUNTS[mesh_name],
                )
            mesh = source_meshes[mesh_name]
            scale = _numbers(mesh_element.get("scale"), (1.0, 1.0, 1.0))
            scale_matrix = np.eye(4)
            scale_matrix[:3, :3] = np.diag(scale)
            transform = root_shift @ transforms[link_name] @ _mesh_visual_transform(link_name, visual) @ scale_matrix
            vertices = trimesh.transform_points(mesh.vertices, transform)
            faces = np.asarray(mesh.faces, dtype=np.int32) + vertex_offset
            all_vertices.append(vertices)
            all_faces.append(faces)
            all_colors.append(np.tile(_part_color(link_name), (len(faces), 1)))
            vertex_offset += len(vertices)

    for leg in ("FR", "FL", "RR", "RL"):
        foot = trimesh.creation.icosphere(subdivisions=1, radius=0.018)
        transform = root_shift @ transforms[f"{leg}_foot"]
        vertices = trimesh.transform_points(foot.vertices, transform)
        faces = np.asarray(foot.faces, dtype=np.int32) + vertex_offset
        all_vertices.append(vertices)
        all_faces.append(faces)
        all_colors.append(np.tile(np.array([22, 22, 22, 255], dtype=np.uint8), (len(faces), 1)))
        vertex_offset += len(vertices)

    vertices = np.vstack(all_vertices).astype(np.float32)
    faces = np.vstack(all_faces).astype(np.int32)
    face_colors = np.vstack(all_colors).astype(np.uint8)
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(CACHE_NPZ, vertices=vertices, faces=faces, face_colors=face_colors)
    assembled = trimesh.Trimesh(vertices=vertices, faces=faces, face_colors=face_colors, process=False)
    assembled.export(CACHE_OBJ)
    return CACHE_NPZ, CACHE_OBJ


@lru_cache(maxsize=2)
def _load_cache(cache_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with np.load(cache_path) as data:
        return data["vertices"], data["faces"], data["face_colors"]


def load_go1_visual(robot_cfg: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cache_path, _ = ensure_go1_visual_cache(robot_cfg)
    return _load_cache(str(cache_path))


@lru_cache(maxsize=1)
def _articulated_visual_template():
    import trimesh

    root = ET.parse(URDF_PATH).getroot()
    nominal_transforms = _link_transforms(root)
    parts = []
    source_meshes = {}
    for link in root.findall("link"):
        link_name = link.get("name", "")
        if link_name not in nominal_transforms:
            continue
        for visual in link.findall("visual"):
            mesh_element = visual.find("geometry/mesh")
            if mesh_element is None:
                continue
            mesh_name = f"{Path(mesh_element.get('filename', '')).stem}.stl"
            if mesh_name not in TARGET_FACE_COUNTS:
                continue
            if mesh_name not in source_meshes:
                source_meshes[mesh_name] = _load_source_mesh(
                    ASSET_ROOT / "simplified_meshes" / mesh_name,
                    TARGET_FACE_COUNTS[mesh_name],
                )
            mesh = source_meshes[mesh_name]
            scale = _numbers(mesh_element.get("scale"), (1.0, 1.0, 1.0))
            scale_matrix = np.eye(4)
            scale_matrix[:3, :3] = np.diag(scale)
            local_transform = _mesh_visual_transform(link_name, visual) @ scale_matrix
            vertices = trimesh.transform_points(mesh.vertices, local_transform).astype(np.float32)
            parts.append((link_name, vertices, np.asarray(mesh.faces, dtype=np.int32), _part_color(link_name)))

    foot = trimesh.creation.icosphere(subdivisions=1, radius=0.018)
    for leg in ("FR", "FL", "RR", "RL"):
        parts.append(
            (
                f"{leg}_foot",
                np.asarray(foot.vertices, dtype=np.float32),
                np.asarray(foot.faces, dtype=np.int32),
                np.array([22, 22, 22, 255], dtype=np.uint8),
            )
        )
    return root, tuple(parts)


def load_go1_articulated_visual(
    robot_cfg: dict,
    joint_positions: dict[str, float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Assemble the Go1 mesh at the supplied 12 joint positions."""
    import trimesh

    root, parts = _articulated_visual_template()
    transforms = _link_transforms(root, joint_positions)
    vertices_all = []
    faces_all = []
    colors_all = []
    vertex_offset = 0
    for link_name, vertices, faces, color in parts:
        world_vertices = trimesh.transform_points(vertices, transforms[link_name])
        vertices_all.append(world_vertices)
        faces_all.append(faces + vertex_offset)
        colors_all.append(np.tile(color, (len(faces), 1)))
        vertex_offset += len(vertices)
    return (
        np.vstack(vertices_all).astype(np.float32),
        np.vstack(faces_all).astype(np.int32),
        np.vstack(colors_all).astype(np.uint8),
    )
