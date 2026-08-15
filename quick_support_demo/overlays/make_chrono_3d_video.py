from __future__ import annotations

import argparse
import copy
from datetime import datetime
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from quick_support_demo.chrono_demo.build_scm_pit import (
    SCMHeightCourse,
    build_rolling_scm_pit,
    build_scm_pit,
    sample_heightmap,
)
from quick_support_demo.chrono_demo.build_support_proxy import build_support_proxy, foot_offsets
from quick_support_demo.chrono_demo.build_world import (
    add_box,
    add_perimeter_floor,
    build_system,
    make_contact_material,
)
from quick_support_demo.chrono_demo.chrono_import import import_chrono
from quick_support_demo.chrono_demo.independent_feet import (
    build_independent_feet,
    contact_adjusted_gait_state,
    update_independent_feet,
)
from quick_support_demo.chrono_demo.difficult_terrain import (
    DifficultCourse,
    RollingCourse,
    add_difficult_course,
    default_difficult_course,
    default_rolling_course,
    support_plane_attitude,
    terrain_adjusted_foot_targets,
)
from quick_support_demo.chrono_demo.hazard import (
    RigidHazard,
    add_rigid_hazard,
    find_hazard_strike,
    flat_heightmap,
    opposite_side_support_indices,
)
from quick_support_demo.config import PROJECT_ROOT, load_demo_config
from quick_support_demo.motion import ForwardTurnForward, TrotGait, VelocityCommand
from quick_support_demo.overlays.pyvista_renderer import FrameContext, PyVistaFrameRenderer
from quick_support_demo.robot_assets.go1 import load_go1_articulated_visual, load_go1_visual


chrono, veh = import_chrono()


def apply_smoke_overrides(cfg: dict) -> None:
    cfg["world"]["world"]["timestep_s"] = 0.001
    cfg["terrain"]["pit"]["grid_spacing_m"] = 0.035


def cuboid_faces(center, size):
    cx, cy, cz = center
    sx, sy, sz = [v / 2.0 for v in size]
    pts = np.array(
        [
            [cx - sx, cy - sy, cz - sz],
            [cx + sx, cy - sy, cz - sz],
            [cx + sx, cy + sy, cz - sz],
            [cx - sx, cy + sy, cz - sz],
            [cx - sx, cy - sy, cz + sz],
            [cx + sx, cy - sy, cz + sz],
            [cx + sx, cy + sy, cz + sz],
            [cx - sx, cy + sy, cz + sz],
        ]
    )
    return [
        [pts[i] for i in [0, 1, 2, 3]],
        [pts[i] for i in [4, 5, 6, 7]],
        [pts[i] for i in [0, 1, 5, 4]],
        [pts[i] for i in [2, 3, 7, 6]],
        [pts[i] for i in [1, 2, 6, 5]],
        [pts[i] for i in [0, 3, 7, 4]],
    ]


def draw_box(ax, center, size, color, alpha=1.0):
    poly = Poly3DCollection(cuboid_faces(center, size), facecolors=color, edgecolors="#202020", linewidths=0.35, alpha=alpha)
    poly.set_zorder(2)
    ax.add_collection3d(poly)


def soil_facecolors(heightmap: np.ndarray, spacing: float) -> np.ndarray:
    """Build a repeatable dirt albedo with lighting driven by the SCM surface."""
    rows, cols = np.indices(heightmap.shape)
    coarse_grain = np.sin(cols * 1.73 + rows * 2.37) * 0.5 + np.sin(cols * 4.11 - rows * 1.29) * 0.5
    fine_grain = np.sin(cols * 9.17 + rows * 7.31)
    grain = 0.07 * coarse_grain + 0.035 * fine_grain

    sinkage = np.clip(-heightmap / 0.055, 0.0, 1.0)
    dry_soil = np.array([0.46, 0.30, 0.15])
    compacted_soil = np.array([0.24, 0.14, 0.075])
    albedo = dry_soil[None, None, :] * (1.0 - sinkage[..., None])
    albedo += compacted_soil[None, None, :] * sinkage[..., None]
    albedo *= (1.0 + grain[..., None])

    slope_y, slope_x = np.gradient(heightmap, spacing, spacing)
    normals = np.stack((-slope_x, -slope_y, np.ones_like(heightmap)), axis=-1)
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)
    light = np.array([-0.45, -0.35, 0.82])
    light /= np.linalg.norm(light)
    diffuse = np.clip(normals @ light, 0.0, 1.0)
    illumination = 0.46 + 0.54 * diffuse
    return np.clip(albedo * illumination[..., None], 0.0, 1.0)


def deformation_stats(heightmap: np.ndarray) -> tuple[float, float, int]:
    # SCM's outermost queried ring reports the pit base rather than soil
    # deformation, so deformation metrics intentionally use interior nodes.
    interior_sinkage = np.maximum(-heightmap[1:-1, 1:-1], 0.0)
    active = interior_sinkage[interior_sinkage > 1.0e-5]
    if active.size == 0:
        return 0.0, 0.0, 0
    return float(np.mean(active)), float(np.max(active)), int(active.size)


def deformation_difference_stats(
    initial_heightmap: np.ndarray,
    heightmap: np.ndarray,
) -> tuple[float, float, int]:
    difference = initial_heightmap[1:-1, 1:-1] - heightmap[1:-1, 1:-1]
    active = difference[difference > 1.0e-5]
    if active.size == 0:
        return 0.0, 0.0, 0
    return float(np.mean(active)), float(np.max(active)), int(active.size)


def dem_difference(initial_heightmap: np.ndarray, heightmap: np.ndarray) -> np.ndarray:
    difference_mm = (heightmap - initial_heightmap).astype(np.float64) * 1000.0
    # The SCM patch boundary reports its base level rather than a deforming
    # surface node. Mask it so it cannot dominate the DEM comparison.
    difference_mm[[0, -1], :] = np.nan
    difference_mm[:, [0, -1]] = np.nan
    return difference_mm


def _quaternion_matrix(rotation) -> np.ndarray:
    w, x, y, z = float(rotation.e0), float(rotation.e1), float(rotation.e2), float(rotation.e3)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def draw_go1_visual(ax, body, robot_cfg, alpha: float = 1.0, joint_positions=None) -> None:
    if joint_positions is None:
        vertices, faces, face_colors = load_go1_visual(robot_cfg)
    else:
        vertices, faces, face_colors = load_go1_articulated_visual(robot_cfg, joint_positions)
    pos = body.GetPos()
    rotation = _quaternion_matrix(body.GetRot())
    world_vertices = vertices @ rotation.T + np.array([pos.x, pos.y, pos.z])
    triangles = world_vertices[faces]

    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.maximum(lengths, 1.0e-12)
    light = np.array([-0.45, -0.35, 0.82])
    light /= np.linalg.norm(light)
    diffuse = np.clip(normals @ light, 0.0, 1.0)
    colors = face_colors[:, :3].astype(float) / 255.0
    colors *= (0.58 + 0.42 * diffuse[:, None])
    collection = Poly3DCollection(
        triangles,
        facecolors=np.clip(colors, 0.0, 1.0),
        edgecolors="none",
        linewidths=0,
        alpha=alpha,
    )
    collection.set_zorder(20)
    ax.add_collection3d(collection)


def draw_dem_panel(
    ax,
    fig,
    initial_heightmap,
    heightmap,
    robot_cfg,
    body,
    max_change_mm,
    gait_state=None,
    contact_feet=None,
    hazard=None,
    rolling_course=None,
):
    extent = (-0.6, 0.6, -0.6, 0.6)
    dem_values = dem_difference(initial_heightmap, heightmap)
    image = ax.imshow(
        dem_values,
        origin="lower",
        extent=extent,
        cmap="RdBu",
        norm=TwoSlopeNorm(vmin=-max_change_mm, vcenter=0.0, vmax=max_change_mm / 3.0),
        interpolation="nearest",
    )

    finite = np.isfinite(dem_values)
    subsidence = -dem_values[finite & (dem_values < -0.01)]
    uplift = dem_values[finite & (dem_values > 0.01)]
    mean_subsidence = float(np.mean(subsidence)) if subsidence.size else 0.0
    max_subsidence = float(np.max(subsidence)) if subsidence.size else 0.0
    max_uplift = float(np.max(uplift)) if uplift.size else 0.0

    if max_subsidence >= 1.0:
        levels = [-level for level in (1.0, 5.0, 10.0, 20.0, 30.0) if level < max_change_mm]
        if levels:
            xs = np.linspace(extent[0], extent[1], dem_values.shape[1])
            ys = np.linspace(extent[2], extent[3], dem_values.shape[0])
            ax.contour(xs, ys, dem_values, levels=sorted(levels), colors="#4b1212", linewidths=0.7, alpha=0.75)

    pos = body.GetPos()
    rot = body.GetRot()
    if rolling_course is not None and gait_state is not None:
        foot_positions = [
            pos + rot.Rotate(chrono.ChVector3d(*offset))
            for offset in gait_state.foot_positions_body.values()
        ]
    elif contact_feet is not None:
        foot_positions = [foot.GetPos() for foot in contact_feet.bodies.values()]
    else:
        offsets = foot_offsets(robot_cfg) if gait_state is None else gait_state.foot_positions_body.values()
        foot_positions = [pos + rot.Rotate(chrono.ChVector3d(*offset)) for offset in offsets]
    for foot in foot_positions:
        if extent[0] <= foot.x <= extent[1] and extent[2] <= foot.y <= extent[3]:
            ax.plot(foot.x, foot.y, marker="x", markersize=7, markeredgewidth=1.4, color="#111111")

    if hazard is not None:
        xmin, xmax, ymin, ymax = hazard.bounds_xy
        ax.add_patch(
            Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                facecolor="#d33a13",
                edgecolor="#7c1606",
                linewidth=1.5,
                alpha=0.55,
            )
        )

    ax.set_title("DEM difference", fontsize=15, weight="bold", pad=12)
    ax.text(
        0.5,
        1.01,
        "current surface - initial surface",
        transform=ax.transAxes,
        ha="center",
        va="bottom",
        fontsize=10,
        color="#444444",
    )
    ax.text(
        0.03,
        0.04,
        f"mean subsidence: {mean_subsidence:0.1f} mm\n"
        f"max subsidence: {max_subsidence:0.1f} mm\n"
        f"max uplift: {max_uplift:0.1f} mm",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=10,
        color="white",
        bbox={"facecolor": "#161616", "alpha": 0.78, "pad": 6, "edgecolor": "none"},
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.grid(False)
    colorbar = fig.colorbar(image, ax=ax, orientation="horizontal", fraction=0.055, pad=0.10)
    colorbar.set_label("surface elevation change (mm): red = subsidence, blue = uplift")


def draw_scene(
    ax,
    heightmap,
    robot_cfg,
    body,
    sim_time,
    selected_text,
    focus_alpha,
    reveal_alpha,
    gait_state=None,
    traversal=False,
    hazard=None,
    hazard_triggered=False,
    hazard_strike_leg=None,
    hazard_slip_distance_m=0.0,
    difficult_course: DifficultCourse | RollingCourse | None = None,
    side_view=False,
    initial_heightmap=None,
):
    pit_size_x, pit_size_y = 1.2, 1.2
    xs = np.linspace(-0.5 * pit_size_x, 0.5 * pit_size_x, heightmap.shape[1])
    ys = np.linspace(-0.5 * pit_size_y, 0.5 * pit_size_y, heightmap.shape[0])
    xx, yy = np.meshgrid(xs, ys)
    spacing = pit_size_x / max(heightmap.shape[1] - 1, 1)
    display_heightmap = heightmap.copy()
    vertical_exaggeration = 1.0 + 2.0 * reveal_alpha
    display_heightmap[1:-1, 1:-1] *= vertical_exaggeration

    surface_colors = soil_facecolors(display_heightmap, spacing)
    if hazard is not None or isinstance(difficult_course, DifficultCourse):
        surface_colors = np.broadcast_to(
            np.array([0.39, 0.41, 0.42]),
            (*display_heightmap.shape, 3),
        )
    soil_surface = ax.plot_surface(
        xx,
        yy,
        display_heightmap,
        facecolors=surface_colors,
        linewidth=0,
        antialiased=False,
        shade=False,
    )
    soil_surface.set_zorder(1)

    # Rigid perimeter floor as four flat boxes.
    draw_box(ax, (-1.05, 0.0, -0.041), (0.9, 3.0, 0.08), "#707476")
    draw_box(ax, (1.05, 0.0, -0.041), (0.9, 3.0, 0.08), "#707476")
    draw_box(ax, (0.0, -1.05, -0.041), (1.2, 0.9, 0.08), "#707476")
    draw_box(ax, (0.0, 1.05, -0.041), (1.2, 0.9, 0.08), "#707476")
    draw_box(ax, (0.0, 1.38, 0.22), (0.6, 0.04, 0.36), "#e4df22")
    if hazard is not None:
        draw_box(
            ax,
            (hazard.center_x_m, hazard.center_y_m, 0.5 * hazard.height_m),
            (hazard.size_x_m, hazard.size_y_m, hazard.height_m),
            "#d33a13",
        )
    if isinstance(difficult_course, DifficultCourse):
        colors = ("#dc741b", "#2e7ca7", "#b7a023")
        for pad, color in zip(difficult_course.pads, colors, strict=True):
            draw_box(
                ax,
                (pad.center_x_m, pad.center_y_m, 0.5 * pad.height_m),
                (pad.size_x_m, pad.size_y_m, pad.height_m),
                color,
            )

    robot = robot_cfg["robot"]
    pos = body.GetPos()
    robot_alpha = 1.0 - 0.72 * reveal_alpha
    if robot.get("visual_asset") == "go1_urdf":
        joints = gait_state.joint_positions if gait_state is not None else None
        draw_go1_visual(ax, body, robot_cfg, robot_alpha, joints)
    else:
        body_size = tuple(float(v) for v in robot["body_size_m"])
        color = "#0d4fd8" if robot["name"] == "go1" else "#e28b1f"
        draw_box(ax, (pos.x, pos.y, pos.z), body_size, color, alpha=0.92)

        rot = body.GetRot()
        foot_h = float(robot["foot_height_m"])
        foot_r = float(robot["foot_radius_m"])
        foot_side = np.sqrt(np.pi * foot_r * foot_r)
        for off in foot_offsets(robot_cfg):
            foot = chrono.ChVector3d(*off)
            world = pos + rot.Rotate(foot)
            draw_box(ax, (world.x, world.y, world.z), (foot_side, foot_side, foot_h), "#111111", alpha=1.0)

    motion_name = "velocity-command trot" if traversal else "scripted approach"
    if hazard is not None:
        terrain_name = "rigid hazard course"
    elif isinstance(difficult_course, RollingCourse):
        terrain_name = "rolling hills + valleys"
    elif difficult_course is not None:
        terrain_name = "rigid difficult course"
    else:
        terrain_name = "Chrono SCM soil"
    ax.text2D(0.03, 0.94, f"{robot['name'].upper()} {motion_name} + {terrain_name}", transform=ax.transAxes, fontsize=13, weight="bold")
    ax.text2D(0.03, 0.89, selected_text, transform=ax.transAxes, fontsize=11)
    ax.text2D(0.03, 0.84, f"t = {sim_time:0.2f} s", transform=ax.transAxes, fontsize=11)
    if hazard is not None or isinstance(difficult_course, DifficultCourse):
        ax.text2D(0.03, 0.79, "Rigid course deformation: none", transform=ax.transAxes, fontsize=10)
    elif isinstance(difficult_course, RollingCourse) and initial_heightmap is not None:
        mean_sinkage, max_sinkage, active_nodes = deformation_difference_stats(
            initial_heightmap, heightmap
        )
        ax.text2D(
            0.03,
            0.79,
            f"SCM displaced nodes: mean {mean_sinkage * 1000:0.1f} mm  |  max {max_sinkage * 1000:0.1f} mm  |  nodes {active_nodes}",
            transform=ax.transAxes,
            fontsize=10,
        )
    else:
        mean_sinkage, max_sinkage, active_nodes = deformation_stats(heightmap)
        ax.text2D(
            0.03,
            0.79,
            f"SCM displaced nodes: mean {mean_sinkage * 1000:0.1f} mm  |  max {max_sinkage * 1000:0.1f} mm  |  nodes {active_nodes}",
            transform=ax.transAxes,
            fontsize=10,
        )
    if traversal:
        ax.text2D(
            0.03,
            0.74,
            (
                "Open-loop gait; geometric toe-strike trigger"
                if hazard is not None
                else "Open-loop gait; trunk follows fitted support plane"
                if difficult_course is not None
                else "Open-loop gait; independent stance feet drive SCM contact"
            ),
            transform=ax.transAxes,
            fontsize=9,
            color="#7b3f00",
            weight="bold",
        )
    if hazard is not None:
        hazard_text = "RIGID OFFSET SLIP HAZARD: awaiting foot contact"
        if hazard_triggered:
            hazard_text = (
                f"TOE CONTACT ({hazard_strike_leg}): one-side support lost\n"
                f"lateral skid {hazard_slip_distance_m:0.2f} m\n"
                "Reduced-order skid + rigid-body fall; no controller"
            )
        ax.text2D(
            0.03,
            0.68,
            hazard_text,
            transform=ax.transAxes,
            fontsize=9,
            color="#8f2209",
            weight="bold",
        )
    if difficult_course is not None:
        up = body.GetRot().Rotate(chrono.ChVector3d(0.0, 0.0, 1.0))
        tilt_deg = np.degrees(np.arccos(np.clip(float(up.z), -1.0, 1.0)))
        ax.text2D(
            0.03,
            0.68,
            f"current trunk tilt {tilt_deg:0.1f} deg\n"
            "Kinematic terrain following; no balance controller",
            transform=ax.transAxes,
            fontsize=9,
            color="#7b3f00",
            weight="bold",
        )
    if reveal_alpha > 0.05:
        ax.text2D(
            0.03,
            0.74,
            f"Footprint reveal: robot faded; {vertical_exaggeration:0.1f}x terrain vertical exaggeration",
            transform=ax.transAxes,
            fontsize=10,
            color="#7b3f00",
            weight="bold",
        )
    view_extent = 1.35 - 0.45 * focus_alpha - 0.15 * reveal_alpha
    ax.set_xlim(-view_extent, view_extent)
    ax.set_ylim(-view_extent, view_extent)
    ax.set_zlim(-0.16 + 0.04 * focus_alpha, 0.72 - 0.27 * focus_alpha)
    ax.set_box_aspect((1, 1, 0.35 + 0.15 * focus_alpha))
    if side_view:
        ax.view_init(elev=10, azim=0)
    else:
        ax.view_init(elev=27 + 6 * focus_alpha + 18 * reveal_alpha, azim=-55)
    ax.set_axis_off()


def make_video(
    robot_name: str,
    duration: float,
    fps: int,
    output_path: Path,
    smoke: bool,
    mass_scale: float,
    dem_panel: bool,
    dem_max_mm: float,
    traverse: bool,
    vx_mps: float,
    vy_mps: float,
    wz_radps: float,
    gait_frequency_hz: float,
    step_height_m: float,
    renderer: str = "matplotlib",
    width: int = 1280,
    height: int = 720,
    hazard_mode: bool = False,
    hazard_offset_x_m: float = 0.13,
    hazard_height_m: float = 0.13,
    hazard_slip_speed_mps: float = 0.55,
    hazard_tip_rate_radps: float = 0.65,
    difficult_terrain_mode: bool = False,
    difficult_max_tilt_deg: float = 14.0,
    side_view: bool = False,
    rolling_terrain_mode: bool = False,
    forward_turn_forward_mode: bool = False,
    first_forward_distance_m: float = 0.85,
    turn_angle_deg: float = -90.0,
    turn_rate_radps: float = 0.8,
    second_forward_distance_m: float = 0.90,
) -> tuple[float, float, int]:
    cfg = load_demo_config()
    if smoke:
        apply_smoke_overrides(cfg)

    if mass_scale <= 0.0:
        raise ValueError("mass_scale must be positive")
    if dem_max_mm <= 0.0:
        raise ValueError("dem_max_mm must be positive")
    if gait_frequency_hz <= 0.0:
        raise ValueError("gait_frequency_hz must be positive")
    if renderer not in {"matplotlib", "pyvista"}:
        raise ValueError(f"unsupported renderer: {renderer}")
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if renderer == "pyvista" and robot_name != "go1":
        raise ValueError("the PyVista renderer currently supports the Go1 visual asset")
    if traverse and robot_name != "go1":
        raise ValueError("velocity-command traversal currently supports only the articulated Go1 visual")
    if hazard_mode and not traverse:
        raise ValueError("the rigid trip hazard requires --traverse")
    if difficult_terrain_mode and not traverse:
        raise ValueError("difficult terrain requires --traverse")
    if hazard_mode and difficult_terrain_mode:
        raise ValueError("--hazard and --difficult-terrain are mutually exclusive")
    if sum((hazard_mode, difficult_terrain_mode, rolling_terrain_mode)) > 1:
        raise ValueError("--hazard, --difficult-terrain, and --rolling-terrain are mutually exclusive")
    if rolling_terrain_mode and not traverse:
        raise ValueError("rolling terrain requires --traverse")
    if forward_turn_forward_mode and not traverse:
        raise ValueError("forward-turn-forward requires --traverse")
    if forward_turn_forward_mode and hazard_mode:
        raise ValueError("forward-turn-forward is not supported with --hazard")
    if forward_turn_forward_mode and vx_mps <= 0.0:
        raise ValueError("forward-turn-forward requires a positive --vx")
    if first_forward_distance_m < 0.0 or second_forward_distance_m < 0.0:
        raise ValueError("forward maneuver distances must be nonnegative")
    if turn_rate_radps <= 0.0:
        raise ValueError("turn_rate_radps must be positive")
    if difficult_terrain_mode and dem_panel:
        raise ValueError("the DEM-difference panel is not meaningful for rigid difficult terrain")
    if hazard_height_m <= 0.0:
        raise ValueError("hazard_height_m must be positive")
    if hazard_slip_speed_mps < 0.0:
        raise ValueError("hazard_slip_speed_mps must be nonnegative")
    if hazard_tip_rate_radps < 0.0:
        raise ValueError("hazard_tip_rate_radps must be nonnegative")
    if difficult_max_tilt_deg <= 0.0 or difficult_max_tilt_deg >= 45.0:
        raise ValueError("difficult_max_tilt_deg must be between 0 and 45 degrees")
    robot_cfg = copy.deepcopy(cfg["robots"][robot_name])
    robot_cfg["robot"]["mass_kg"] = float(robot_cfg["robot"]["mass_kg"]) * mass_scale
    robot_cfg["robot"]["payload_kg"] = float(robot_cfg["robot"].get("payload_kg", 0.0)) * mass_scale
    candidates = cfg["candidates"]["candidates"]
    system = build_system(cfg["world"])
    add_perimeter_floor(system, cfg["world"], cfg["terrain"])
    hazard = None
    difficult_course = None
    rolling_course = None
    if hazard_mode:
        hazard = RigidHazard(center_x_m=hazard_offset_x_m, height_m=hazard_height_m)
        pit_size_x, pit_size_y = cfg["terrain"]["pit"]["size_m"]
        add_box(
            system,
            "rigid_hazard_course_plate",
            (pit_size_x, pit_size_y, 0.08),
            (0.0, 0.0, -0.04),
            make_contact_material(0.9),
            density=1800.0,
            fixed=True,
            color=(0.36, 0.37, 0.38),
        )
        add_rigid_hazard(system, hazard)
        terrain = None
    elif difficult_terrain_mode:
        difficult_course = default_difficult_course()
        pit_size_x, pit_size_y = cfg["terrain"]["pit"]["size_m"]
        add_box(
            system,
            "rigid_difficult_course_plate",
            (pit_size_x, pit_size_y, 0.08),
            (0.0, 0.0, -0.04),
            make_contact_material(0.82),
            density=1800.0,
            fixed=True,
            color=(0.36, 0.37, 0.38),
        )
        add_difficult_course(system, difficult_course)
        terrain = None
    elif rolling_terrain_mode:
        rolling_course = default_rolling_course()
        terrain = build_rolling_scm_pit(
            system,
            cfg["terrain"],
            rolling_course,
            visualization_mesh=False,
        )
    else:
        terrain = build_scm_pit(system, cfg["terrain"], visualization_mesh=False)

    sand_xy = tuple(candidates["sand"]["base_xy_m"])
    start_xy = (sand_xy[0], -1.10)
    z0 = float(robot_cfg["robot"]["body_com_height_m"]) + 0.02
    retained_support_indices = None
    if hazard_mode:
        retained_support_indices = opposite_side_support_indices(
            foot_offsets(robot_cfg), hazard_offset_x_m
        )
    body = build_support_proxy(
        system,
        robot_cfg,
        start_xy,
        foot_collisions=not traverse or hazard_mode,
        body_collision=hazard_mode,
        contact_friction=0.18 if hazard_mode else 0.9,
        collision_foot_indices=retained_support_indices,
    )
    if traverse:
        body.SetFixed(True)
        body.EnableCollision(False)
        body.SetRot(chrono.QuatFromAngleZ(np.pi / 2.0))
    else:
        body.SetFixed(True)
        body.EnableCollision(False)

    dt = float(cfg["world"]["world"]["timestep_s"])
    approach_time = min(2.6, duration * 0.45)
    next_capture = 0.0
    sim_time = 0.0
    released = False
    traversal_warmup_s = 0.8
    support_course = (
        SCMHeightCourse(
            terrain,
            tuple(float(value) for value in cfg["terrain"]["pit"]["size_m"]),
            float(cfg["terrain"]["pit"]["top_elevation_m"]),
        )
        if rolling_course is not None
        else difficult_course
    )
    visual_course = difficult_course or rolling_course
    traversal_end_y = 0.95 if support_course is not None else 1.10
    commanded_x, commanded_y = start_xy
    commanded_yaw = np.pi / 2.0
    maneuver = (
        ForwardTurnForward(
            vx_mps,
            first_forward_distance_m,
            np.radians(turn_angle_deg),
            turn_rate_radps,
            second_forward_distance_m,
        )
        if forward_turn_forward_mode
        else None
    )
    maneuver_state = maneuver.sample(0.0) if maneuver is not None else None
    gait = TrotGait(
        float(robot_cfg["robot"]["body_com_height_m"]),
        float(robot_cfg["robot"]["foot_height_m"]),
        frequency_hz=gait_frequency_hz,
        step_height_m=step_height_m,
    )
    total_mass = float(robot_cfg["robot"]["mass_kg"]) + float(robot_cfg["robot"].get("payload_kg", 0.0))
    desired_gait_state = gait.sample(0.0, VelocityCommand()) if traverse else None
    visual_gait_state = desired_gait_state
    independent_feet = None
    hazard_triggered = False
    hazard_strike_leg = None
    hazard_trigger_time = None
    hazard_trigger_x = None
    course_roll = 0.0
    course_pitch = 0.0
    course_height = 0.0
    max_course_tilt_deg = 0.0
    if traverse:
        independent_feet = build_independent_feet(
            system,
            robot_cfg,
            body.GetPos(),
            commanded_yaw,
            desired_gait_state,
            total_mass,
        )
    if terrain is None:
        last_heightmap = flat_heightmap(cfg["terrain"])
    else:
        last_heightmap = sample_heightmap(terrain, cfg["terrain"])
    initial_heightmap = last_heightmap.copy()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    grid_spacing_mm = float(cfg["terrain"]["pit"]["grid_spacing_m"]) * 1000.0
    if traverse:
        if maneuver is not None:
            selected_text = (
                f"FTF {first_forward_distance_m:g} m, turn {turn_angle_deg:g} deg @ "
                f"{turn_rate_radps:g} rad/s, {second_forward_distance_m:g} m; "
                f"v={vx_mps:g} m/s; mass {total_mass:g} kg"
            )
        elif hazard_mode:
            selected_text = f"cmd v=({vx_mps:g}, {vy_mps:g}) m/s, w={wz_radps:g} rad/s; mass {total_mass:g} kg; rigid plate"
        elif difficult_terrain_mode:
            selected_text = f"cmd v=({vx_mps:g}, {vy_mps:g}) m/s, w={wz_radps:g} rad/s; mass {total_mass:g} kg; rigid uneven course"
        elif rolling_terrain_mode:
            selected_text = f"cmd v=({vx_mps:g}, {vy_mps:g}) m/s, w={wz_radps:g} rad/s; mass {total_mass:g} kg; rolling Chrono SCM"
        else:
            selected_text = f"cmd v=({vx_mps:g}, {vy_mps:g}) m/s, w={wz_radps:g} rad/s; mass {total_mass:g} kg; grid {grid_spacing_mm:g} mm"
    else:
        selected_text = f"Go1 mesh; physical proxy mass {total_mass:g} kg; SCM grid {grid_spacing_mm:g} mm."

    vtk_renderer = None
    try:
        if renderer == "pyvista":
            vtk_renderer = PyVistaFrameRenderer(
                initial_heightmap,
                robot_cfg,
                width=width,
                height=height,
                dem_panel=dem_panel,
                dem_max_mm=dem_max_mm,
                hazard=hazard,
                difficult_course=difficult_course,
                rolling_course=rolling_course,
                side_view=side_view,
            )
        with imageio.get_writer(output_path, fps=fps, codec="libx264", quality=8) as writer:
            while sim_time < duration:
                command = VelocityCommand()
                if traverse:
                    if not hazard_triggered:
                        if sim_time >= traversal_warmup_s:
                            if maneuver is not None:
                                maneuver_state = maneuver.sample(
                                    sim_time - traversal_warmup_s
                                )
                                command = maneuver_state.command
                            elif commanded_y < traversal_end_y:
                                command = VelocityCommand(vx_mps, vy_mps, wz_radps)
                        c, s = np.cos(commanded_yaw), np.sin(commanded_yaw)
                        world_vx = c * command.vx_mps - s * command.vy_mps
                        world_vy = s * command.vx_mps + c * command.vy_mps
                        commanded_x += world_vx * dt
                        if maneuver is not None:
                            commanded_y += world_vy * dt
                        else:
                            commanded_y = min(
                                commanded_y + world_vy * dt, traversal_end_y
                            )
                        commanded_yaw += command.wz_radps * dt
                        gait_time = max(sim_time - traversal_warmup_s, 0.0)
                        desired_gait_state = gait.sample(gait_time, command)
                        if support_course is not None:
                            target_roll, target_pitch, target_height = support_plane_attitude(
                                desired_gait_state.foot_positions_body,
                                (commanded_x, commanded_y),
                                commanded_yaw,
                                support_course,
                                np.radians(difficult_max_tilt_deg),
                            )
                            smoothing = dt / (0.14 + dt)
                            course_roll += smoothing * (target_roll - course_roll)
                            course_pitch += smoothing * (target_pitch - course_pitch)
                            course_height += smoothing * (target_height - course_height)
                            nominal_z = float(robot_cfg["robot"]["body_com_height_m"]) + float(
                                robot_cfg["robot"]["start_clearance_m"]
                            )
                            body.SetPos(
                                chrono.ChVector3d(commanded_x, commanded_y, nominal_z + course_height)
                            )
                            body.SetRot(
                                chrono.QuatFromAngleZ(commanded_yaw)
                                * chrono.QuatFromAngleX(course_roll)
                                * chrono.QuatFromAngleY(course_pitch)
                            )
                            current_tilt_deg = np.degrees(
                                np.hypot(course_roll, course_pitch)
                            )
                            max_course_tilt_deg = max(max_course_tilt_deg, current_tilt_deg)
                            adjusted_feet = terrain_adjusted_foot_targets(
                                desired_gait_state.foot_positions_body,
                                gait.nominal_foot("FR")[2],
                                np.array(
                                    [
                                        float(body.GetPos().x),
                                        float(body.GetPos().y),
                                        float(body.GetPos().z),
                                    ]
                                ),
                                commanded_yaw,
                                course_roll,
                                course_pitch,
                                support_course,
                                float(robot_cfg["robot"]["foot_height_m"]),
                            )
                            visual_gait_state = gait.state_from_feet(
                                adjusted_feet,
                                desired_gait_state.stance,
                                desired_gait_state.phase,
                            )
                        else:
                            pos = body.GetPos()
                            body.SetPos(chrono.ChVector3d(commanded_x, commanded_y, pos.z))
                            body.SetRot(chrono.QuatFromAngleZ(commanded_yaw))
                        body.SetLinVel(chrono.ChVector3d(world_vx, world_vy, body.GetPosDt().z))
                        body.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, command.wz_radps))
                        update_independent_feet(
                            independent_feet,
                            desired_gait_state,
                            body.GetPos(),
                            commanded_yaw,
                            total_mass,
                            dt,
                        )
                        strike_leg = find_hazard_strike(independent_feet, hazard) if hazard is not None else None
                        if strike_leg is not None:
                            hazard_triggered = True
                            hazard_strike_leg = strike_leg
                            hazard_trigger_time = sim_time
                            hazard_trigger_x = float(body.GetPos().x)
                            body.SetFixed(False)
                            body.EnableCollision(True)
                            tip_sign = 1.0 if hazard.center_x_m >= float(body.GetPos().x) else -1.0
                            body.SetLinVel(
                                chrono.ChVector3d(
                                    hazard_slip_speed_mps * tip_sign,
                                    world_vy,
                                    0.0,
                                )
                            )
                            body.SetAngVelParent(
                                chrono.ChVector3d(0.0, hazard_tip_rate_radps * tip_sign, 0.0)
                            )
                            for foot in independent_feet.bodies.values():
                                foot.SetFixed(True)
                                foot.EnableCollision(False)
                    if terrain is not None:
                        terrain.Synchronize(system.GetChTime())
                    system.DoStepDynamics(dt)
                    if terrain is not None:
                        terrain.Advance(dt)
                elif sim_time < approach_time:
                    alpha = sim_time / max(approach_time, 1.0e-6)
                    ease = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha
                    x = (1.0 - ease) * start_xy[0] + ease * sand_xy[0]
                    y = (1.0 - ease) * start_xy[1] + ease * sand_xy[1]
                    body.SetPos(chrono.ChVector3d(x, y, z0 + 0.02 * (1.0 - ease)))
                    body.SetRot(chrono.QUNIT)
                    system.DoStepDynamics(dt)
                else:
                    if not released:
                        body.SetFixed(False)
                        body.EnableCollision(True)
                        body.SetPos(chrono.ChVector3d(sand_xy[0], sand_xy[1], z0))
                        body.SetLinVel(chrono.ChVector3d(0.0, 0.0, 0.0))
                        body.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 0.0))
                        released = True
                    terrain.Synchronize(system.GetChTime())
                    system.DoStepDynamics(dt)
                    terrain.Advance(dt)

                if sim_time >= next_capture:
                    if terrain is None:
                        heightmap = initial_heightmap
                    else:
                        heightmap = sample_heightmap(terrain, cfg["terrain"])
                    last_heightmap = heightmap
                    if traverse:
                        if support_course is not None:
                            gait_state = visual_gait_state
                        elif hazard_triggered:
                            gait_state = desired_gait_state
                        else:
                            gait_state = contact_adjusted_gait_state(
                                gait,
                                desired_gait_state,
                                independent_feet,
                                body.GetPos(),
                                commanded_yaw,
                            )
                    else:
                        gait_state = None
                    focus_linear = 0.0 if traverse else np.clip((sim_time - approach_time) / 0.8, 0.0, 1.0)
                    focus_alpha = focus_linear * focus_linear * (3.0 - 2.0 * focus_linear)
                    reveal_start = max(approach_time + 1.2, duration - 1.25)
                    reveal_linear = np.clip((sim_time - reveal_start) / 0.7, 0.0, 1.0)
                    reveal_alpha = reveal_linear * reveal_linear * (3.0 - 2.0 * reveal_linear)
                    hazard_slip_distance = (
                        abs(float(body.GetPos().x) - hazard_trigger_x)
                        if hazard_trigger_x is not None
                        else 0.0
                    )

                    if vtk_renderer is not None:
                        frame = vtk_renderer.render(
                            FrameContext(
                                heightmap=heightmap,
                                initial_heightmap=initial_heightmap,
                                robot_cfg=robot_cfg,
                                body=body,
                                sim_time=sim_time,
                                selected_text=selected_text,
                                gait_state=gait_state,
                                contact_feet=independent_feet,
                                traversal=traverse,
                                reveal_alpha=0.0 if (dem_panel or hazard_mode or support_course is not None) else reveal_alpha,
                                hazard_triggered=hazard_triggered,
                                hazard_strike_leg=hazard_strike_leg,
                                hazard_slip_distance_m=hazard_slip_distance,
                                maneuver_phase=(
                                    "warmup"
                                    if maneuver_state is not None
                                    and sim_time < traversal_warmup_s
                                    else maneuver_state.phase
                                    if maneuver_state is not None
                                    else None
                                ),
                            )
                        )
                    else:
                        fig = plt.figure(figsize=(width / 100.0, height / 100.0), dpi=100)
                        fig.patch.set_facecolor("#e8ecef")
                        if dem_panel:
                            grid = fig.add_gridspec(
                                1,
                                2,
                                width_ratios=(1.55, 1.0),
                                left=0.015,
                                right=0.98,
                                bottom=0.10,
                                top=0.98,
                                wspace=0.04,
                            )
                            scene_ax = fig.add_subplot(grid[0, 0], projection="3d", computed_zorder=False)
                            dem_ax = fig.add_subplot(grid[0, 1])
                            draw_scene(
                                scene_ax,
                                heightmap,
                                robot_cfg,
                                body,
                                sim_time,
                                selected_text,
                                focus_alpha,
                                0.0,
                                gait_state,
                                traverse,
                                hazard,
                                hazard_triggered,
                                hazard_strike_leg,
                                hazard_slip_distance,
                                visual_course,
                                side_view,
                                initial_heightmap,
                            )
                            draw_dem_panel(
                                dem_ax,
                                fig,
                                initial_heightmap,
                                heightmap,
                                robot_cfg,
                                body,
                                dem_max_mm,
                                gait_state,
                                independent_feet,
                                hazard,
                                rolling_course,
                            )
                        else:
                            scene_ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
                            draw_scene(
                                scene_ax,
                                heightmap,
                                robot_cfg,
                                body,
                                sim_time,
                                selected_text,
                                focus_alpha,
                                0.0 if (hazard_mode or support_course is not None) else reveal_alpha,
                                gait_state,
                                traverse,
                                hazard,
                                hazard_triggered,
                                hazard_strike_leg,
                                hazard_slip_distance,
                                visual_course,
                                side_view,
                                initial_heightmap,
                            )
                        fig.canvas.draw()
                        frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
                        plt.close(fig)
                    writer.append_data(frame)
                    next_capture += 1.0 / fps

                sim_time += dt
    finally:
        if vtk_renderer is not None:
            vtk_renderer.close()

    if hazard_mode:
        if hazard_trigger_time is None:
            print("Hazard event: no foot strike detected")
        else:
            angles = body.GetRot().GetCardanAnglesXYZ()
            up = body.GetRot().Rotate(chrono.ChVector3d(0.0, 0.0, 1.0))
            tilt_deg = np.degrees(np.arccos(np.clip(float(up.z), -1.0, 1.0)))
            print(
                f"Hazard event: {hazard_strike_leg} strike at t={hazard_trigger_time:.3f} s; "
                f"final tilt={tilt_deg:.1f} deg, roll={np.degrees(angles.x):.1f} deg, "
                f"pitch={np.degrees(angles.y):.1f} deg, "
                f"lateral skid={abs(float(body.GetPos().x) - hazard_trigger_x):.3f} m"
            )
    if difficult_terrain_mode:
        completed = (
            maneuver_state.completed
            if maneuver_state is not None
            else commanded_y >= traversal_end_y - 1.0e-6
        )
        print(
            f"Difficult terrain: completed={completed}; final y={commanded_y:.3f} m; "
            f"maximum commanded trunk tilt={max_course_tilt_deg:.1f} deg"
        )
    if rolling_terrain_mode:
        completed = (
            maneuver_state.completed
            if maneuver_state is not None
            else commanded_y >= traversal_end_y - 1.0e-6
        )
        rolling_mean, rolling_max, rolling_nodes = deformation_difference_stats(
            initial_heightmap, last_heightmap
        )
        print(
            f"Rolling terrain: completed={completed}; final y={commanded_y:.3f} m; "
            f"maximum commanded trunk tilt={max_course_tilt_deg:.1f} deg; "
            f"elevation range=[{float(np.min(initial_heightmap)):.3f}, "
            f"{float(np.max(initial_heightmap)):.3f}] m; "
            f"deformation mean={rolling_mean * 1000:.2f} mm, "
            f"max={rolling_max * 1000:.2f} mm, nodes={rolling_nodes}"
        )
    if maneuver is not None:
        print(
            f"Forward-turn-forward: completed={maneuver_state.completed}; "
            f"final position=({commanded_x:.3f}, {commanded_y:.3f}) m; "
            f"final yaw={np.degrees(commanded_yaw):.1f} deg"
        )

    if rolling_terrain_mode:
        return deformation_difference_stats(initial_heightmap, last_heightmap)
    return deformation_stats(last_heightmap)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a 3D Chrono-driven proxy demo video.")
    parser.add_argument("--robot", choices=["go1", "spot"], default="go1")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--mass-scale", type=float, default=1.0, help="Scale robot mass to make SCM deformation more visible.")
    parser.add_argument(
        "--dem-panel",
        action="store_true",
        help="Render the 3D scene beside a current-minus-initial SCM deformation DEM.",
    )
    parser.add_argument(
        "--dem-max-mm",
        type=float,
        default=40.0,
        help="Fixed SCM-difference color limit in millimeters.",
    )
    parser.add_argument("--traverse", action="store_true", help="Animate a velocity-command trot across the pit.")
    parser.add_argument("--vx", type=float, default=0.25, help="Commanded forward body velocity in m/s.")
    parser.add_argument("--vy", type=float, default=0.0, help="Commanded lateral body velocity in m/s.")
    parser.add_argument("--wz", type=float, default=0.0, help="Commanded yaw rate in rad/s.")
    parser.add_argument("--gait-frequency", type=float, default=1.6, help="Open-loop trot frequency in Hz.")
    parser.add_argument("--step-height", type=float, default=0.055, help="Swing-foot clearance in meters.")
    parser.add_argument("--renderer", choices=("matplotlib", "pyvista"), default="matplotlib")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--hazard", action="store_true", help="Add an offset rigid obstacle and release a supported trunk into a lateral skid after foot contact.")
    parser.add_argument("--hazard-offset-x", type=float, default=0.13, help="Rigid hazard center x offset in meters.")
    parser.add_argument("--hazard-height", type=float, default=0.13, help="Rigid hazard height in meters.")
    parser.add_argument("--hazard-slip-speed", type=float, default=0.55, help="Lateral release speed after hazard contact in m/s.")
    parser.add_argument("--hazard-tip-rate", type=float, default=0.65, help="Initial trunk tip rate after hazard contact in rad/s.")
    parser.add_argument("--difficult-terrain", action="store_true", help="Traverse a rigid uneven course while fitting trunk attitude to foot support heights.")
    parser.add_argument("--difficult-max-tilt-deg", type=float, default=14.0, help="Maximum commanded roll or pitch on uneven rigid terrain.")
    parser.add_argument("--side-view", action="store_true", help="Render a lateral camera view for foot-height inspection.")
    parser.add_argument("--rolling-terrain", action="store_true", help="Traverse deformable Chrono SCM initialized with smooth hills and valleys.")
    parser.add_argument(
        "--forward-turn-forward",
        action="store_true",
        help="Run a forward, in-place turn, forward velocity-command sequence.",
    )
    parser.add_argument("--first-forward-distance", type=float, default=0.85, help="First forward segment distance in meters.")
    parser.add_argument("--turn-angle-deg", type=float, default=-90.0, help="Turn angle in degrees; negative turns right.")
    parser.add_argument("--turn-rate", type=float, default=0.8, help="Absolute in-place turn rate in rad/s.")
    parser.add_argument("--second-forward-distance", type=float, default=0.90, help="Second forward segment distance in meters.")
    resolution = parser.add_mutually_exclusive_group()
    resolution.add_argument("--smoke", dest="smoke", action="store_true", help="Use the coarse 35 mm SCM grid.")
    resolution.add_argument("--full-res", dest="smoke", action="store_false", help="Use the configured SCM grid (default).")
    parser.set_defaults(smoke=False)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    load_suffix = "" if args.mass_scale == 1.0 else f"_{args.mass_scale:g}x_mass"
    dem_suffix = "_dem" if args.dem_panel else ""
    traversal_suffix = "_traverse" if args.traverse else ""
    hazard_suffix = "_hazard" if args.hazard else ""
    difficult_suffix = "_difficult" if args.difficult_terrain else ""
    side_suffix = "_side" if args.side_view else ""
    rolling_suffix = "_rolling" if args.rolling_terrain else ""
    maneuver_suffix = "_forward_turn_forward" if args.forward_turn_forward else ""
    output = args.output or PROJECT_ROOT / "quick_support_demo" / "outputs" / "videos" / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{args.robot}_chrono_3d{load_suffix}{dem_suffix}{traversal_suffix}{hazard_suffix}{difficult_suffix}{rolling_suffix}{maneuver_suffix}{side_suffix}.mp4"
    mean_sinkage, max_sinkage, active_nodes = make_video(
        args.robot,
        args.duration,
        args.fps,
        output.resolve(),
        args.smoke,
        args.mass_scale,
        args.dem_panel,
        args.dem_max_mm,
        args.traverse,
        args.vx,
        args.vy,
        args.wz,
        args.gait_frequency,
        args.step_height,
        args.renderer,
        args.width,
        args.height,
        args.hazard,
        args.hazard_offset_x,
        args.hazard_height,
        args.hazard_slip_speed,
        args.hazard_tip_rate,
        args.difficult_terrain,
        args.difficult_max_tilt_deg,
        args.side_view,
        args.rolling_terrain,
        args.forward_turn_forward,
        args.first_forward_distance,
        args.turn_angle_deg,
        args.turn_rate,
        args.second_forward_distance,
    )
    print(output.resolve())
    if args.hazard:
        print("Rigid hazard course deformation: none")
    elif args.difficult_terrain:
        print("Rigid difficult course deformation: none")
    elif args.rolling_terrain:
        print(
            f"Final rolling SCM deformation: mean={mean_sinkage * 1000:.2f} mm, "
            f"max={max_sinkage * 1000:.2f} mm, nodes={active_nodes}"
        )
    else:
        print(f"Final interior SCM deformation: mean={mean_sinkage * 1000:.2f} mm, max={max_sinkage * 1000:.2f} mm, nodes={active_nodes}")


if __name__ == "__main__":
    main()
