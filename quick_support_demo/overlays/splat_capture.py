from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import imageio.v2 as imageio
import numpy as np

from quick_support_demo.overlays.pyvista_renderer import FrameContext, PyVistaFrameRenderer


@dataclass(frozen=True)
class OrbitView:
    index: int
    theta_deg: float
    phi_deg: float
    position: tuple[float, float, float]


def parse_float_list(value: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as exc:
        raise ValueError(f"expected a comma-separated list of numbers, got {value!r}") from exc
    if not values:
        raise ValueError("angle list must not be empty")
    return values


def parse_xyz(value: str) -> tuple[float, float, float]:
    values = parse_float_list(value)
    if len(values) != 3:
        raise ValueError(f"expected x,y,z, got {value!r}")
    return values


def orbit_views(
    theta_degrees: Iterable[float],
    *,
    radius_m: float,
    target: tuple[float, float, float],
    phi_degrees: Iterable[float] | None = None,
    phi_count: int = 36,
    phi_offset_deg: float = 0.0,
) -> list[OrbitView]:
    """Build Z-up spherical camera rings; theta is elevation above the XY plane."""
    theta_values = tuple(float(value) for value in theta_degrees)
    if not theta_values:
        raise ValueError("at least one theta level is required")
    if any(abs(theta) >= 89.0 for theta in theta_values):
        raise ValueError("theta levels must be between -89 and 89 degrees")
    if radius_m <= 0.0:
        raise ValueError("orbit radius must be positive")

    if phi_degrees is None:
        if phi_count < 1:
            raise ValueError("phi_count must be at least one")
        phi_values = tuple(
            float(phi_offset_deg + index * 360.0 / phi_count) for index in range(phi_count)
        )
    else:
        phi_values = tuple(float(value) for value in phi_degrees)
        if not phi_values:
            raise ValueError("at least one phi angle is required")

    target_vector = np.asarray(target, dtype=float)
    views: list[OrbitView] = []
    for theta in theta_values:
        theta_rad = np.radians(theta)
        for phi in phi_values:
            phi_rad = np.radians(phi)
            offset = radius_m * np.array(
                [
                    np.cos(theta_rad) * np.cos(phi_rad),
                    np.cos(theta_rad) * np.sin(phi_rad),
                    np.sin(theta_rad),
                ]
            )
            position = target_vector + offset
            views.append(
                OrbitView(
                    index=len(views),
                    theta_deg=theta,
                    phi_deg=phi % 360.0,
                    position=tuple(float(value) for value in position),
                )
            )
    return views


def camera_to_world_opengl(
    position: tuple[float, float, float],
    target: tuple[float, float, float],
    world_up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> np.ndarray:
    """Return an OpenGL/NeRF camera-to-world matrix (right, up, back, position)."""
    position_vector = np.asarray(position, dtype=float)
    forward = np.asarray(target, dtype=float) - position_vector
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray(world_up, dtype=float))
    right_norm = np.linalg.norm(right)
    if right_norm < 1.0e-8:
        raise ValueError("camera direction is parallel to world_up")
    right /= right_norm
    back = -forward
    camera_up = np.cross(back, right)

    transform = np.eye(4, dtype=float)
    transform[:3, 0] = right
    transform[:3, 1] = camera_up
    transform[:3, 2] = back
    transform[:3, 3] = position_vector
    return transform


def pinhole_intrinsics(width: int, height: int, view_angle_deg: float) -> dict[str, float | int]:
    """Compute square-pixel intrinsics from VTK's vertical perspective field of view."""
    if width <= 0 or height <= 0:
        raise ValueError("capture width and height must be positive")
    if view_angle_deg <= 0.0 or view_angle_deg >= 180.0:
        raise ValueError("view angle must be between 0 and 180 degrees")
    focal = 0.5 * height / np.tan(0.5 * np.radians(view_angle_deg))
    return {
        "w": int(width),
        "h": int(height),
        "fl_x": float(focal),
        "fl_y": float(focal),
        "cx": float(width) / 2.0,
        "cy": float(height) / 2.0,
    }


def metric_depth_to_uint16_mm(depth_m: np.ndarray) -> np.ndarray:
    """Encode positive metric depth as millimeters; zero marks invalid pixels."""
    depth_m = np.asarray(depth_m, dtype=float)
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
    scaled = np.clip(np.rint(depth_m[valid] * 1000.0), 1.0, 65535.0)
    depth_mm[valid] = scaled.astype(np.uint16)
    return depth_mm


def capture_orbit_dataset(
    renderer: PyVistaFrameRenderer,
    context: FrameContext,
    output_dir: Path,
    *,
    theta_degrees: Iterable[float],
    radius_m: float,
    target: tuple[float, float, float],
    view_angle_deg: float,
    phi_degrees: Iterable[float] | None = None,
    phi_count: int = 36,
    phi_offset_deg: float = 0.0,
) -> int:
    """Render a frozen simulation state and write a transforms-based RGB-D dataset."""
    views = orbit_views(
        theta_degrees,
        radius_m=radius_m,
        target=target,
        phi_degrees=phi_degrees,
        phi_count=phi_count,
        phi_offset_deg=phi_offset_deg,
    )
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    image_dir = output_dir / "images"
    image_dir.mkdir()
    depth_dir = output_dir / "depth"
    depth_dir.mkdir()
    depth_png_dir = output_dir / "depth_png"
    depth_png_dir.mkdir()

    intrinsics = pinhole_intrinsics(renderer.width, renderer.height, view_angle_deg)
    transform_frames = []
    camera_frames = []
    for view in views:
        renderer.set_camera(view.position, target, view_angle_deg)
        rgb = renderer.render(context, include_hud=False)
        depth_m = renderer.depth_image_m()
        if depth_m.shape != rgb.shape[:2]:
            raise RuntimeError(
                f"RGB/depth shape mismatch: RGB {rgb.shape[:2]}, depth {depth_m.shape}"
            )
        image_name = f"frame_{view.index:05d}.png"
        depth_name = f"frame_{view.index:05d}.npy"
        depth_png_name = f"frame_{view.index:05d}.png"
        imageio.imwrite(image_dir / image_name, np.asarray(rgb, dtype=np.uint8))
        np.save(depth_dir / depth_name, np.asarray(depth_m, dtype=np.float32))
        imageio.imwrite(depth_png_dir / depth_png_name, metric_depth_to_uint16_mm(depth_m))

        camera_to_world = camera_to_world_opengl(view.position, target)
        world_to_camera = np.linalg.inv(camera_to_world)
        shared = {
            "frame_id": view.index,
            "file_path": f"images/{image_name}",
            "depth_file_path": f"depth_png/{depth_png_name}",
            "depth_npy_file_path": f"depth/{depth_name}",
            "theta_deg": view.theta_deg,
            "phi_deg": view.phi_deg,
            "radius_m": float(radius_m),
        }
        transform_frames.append(
            {
                **shared,
                "transform_matrix": camera_to_world.tolist(),
            }
        )
        camera_frames.append(
            {
                **shared,
                "position_world_m": list(view.position),
                "target_world_m": list(target),
                "camera_to_world_opengl": camera_to_world.tolist(),
                "world_to_camera_opengl": world_to_camera.tolist(),
            }
        )

    camera_angle_x = 2.0 * np.arctan(intrinsics["w"] / (2.0 * intrinsics["fl_x"]))
    camera_angle_y = 2.0 * np.arctan(intrinsics["h"] / (2.0 * intrinsics["fl_y"]))
    transforms = {
        "camera_model": "OPENCV",
        **intrinsics,
        "k1": 0.0,
        "k2": 0.0,
        "p1": 0.0,
        "p2": 0.0,
        "camera_angle_x": float(camera_angle_x),
        "camera_angle_y": float(camera_angle_y),
        "depth_unit_scale_factor": 0.001,
        "frames": transform_frames,
    }
    cameras = {
        "schema_version": 1,
        "coordinate_system": {
            "world": "right-handed, Z up",
            "camera": "OpenGL/NeRF: +X right, +Y up, -Z view direction",
            "transform_matrix": "camera-to-world",
            "theta": "elevation in degrees above the world XY plane",
            "phi": "azimuth in degrees from world +X toward world +Y",
        },
        "intrinsics": intrinsics,
        "view_angle_deg_vertical": float(view_angle_deg),
        "capture_target_world_m": list(target),
        "simulation_time_s": float(context.sim_time),
        "robot_rendered": bool(renderer.show_robot),
        "rgb": {
            "format": "PNG",
            "channels": "RGB",
            "dtype": "uint8",
        },
        "depth": {
            "geometry": "camera-axis Z depth, not Euclidean ray range",
            "npy": {
                "format": "NumPy .npy",
                "units": "meters",
                "dtype": "float32",
                "invalid_value": "NaN",
            },
            "png": {
                "format": "PNG",
                "units": "millimeters",
                "dtype": "uint16",
                "invalid_value": 0,
                "meters_per_unit": 0.001,
            },
        },
        "frames": camera_frames,
    }
    (output_dir / "transforms.json").write_text(json.dumps(transforms, indent=2) + "\n")
    (output_dir / "cameras.json").write_text(json.dumps(cameras, indent=2) + "\n")
    return len(views)
