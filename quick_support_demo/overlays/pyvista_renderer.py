from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

from quick_support_demo.chrono_demo.build_support_proxy import foot_offsets
from quick_support_demo.chrono_demo.chrono_import import import_chrono
from quick_support_demo.chrono_demo.hazard import RigidHazard
from quick_support_demo.chrono_demo.difficult_terrain import DifficultCourse, RollingCourse
from quick_support_demo.robot_assets.go1 import load_go1_articulated_visual, load_go1_visual


PIT_SIZE_M = (1.2, 1.2)
chrono, _ = import_chrono()


def _vtk_faces(faces: np.ndarray) -> np.ndarray:
    faces = np.asarray(faces, dtype=np.int64)
    counts = np.full((len(faces), 1), faces.shape[1], dtype=np.int64)
    return np.hstack((counts, faces)).ravel()


def surface_mesh_arrays(
    heightmap: np.ndarray,
    size_m: tuple[float, float] = PIT_SIZE_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Return points and quad connectivity for a regular SCM height map."""
    rows, cols = heightmap.shape
    xs = np.linspace(-0.5 * size_m[0], 0.5 * size_m[0], cols)
    ys = np.linspace(-0.5 * size_m[1], 0.5 * size_m[1], rows)
    xx, yy = np.meshgrid(xs, ys)
    points = np.column_stack((xx.ravel(), yy.ravel(), heightmap.ravel()))

    upper_left = np.arange((rows - 1) * (cols - 1), dtype=np.int64)
    upper_left += upper_left // (cols - 1)
    faces = np.column_stack(
        (
            upper_left,
            upper_left + 1,
            upper_left + cols + 1,
            upper_left + cols,
        )
    )
    return points, faces


def soil_point_colors(heightmap: np.ndarray) -> np.ndarray:
    """Create deterministic granular albedo without baking in scene lighting."""
    rows, cols = np.indices(heightmap.shape)
    grain = 0.55 * np.sin(cols * 1.73 + rows * 2.37)
    grain += 0.30 * np.sin(cols * 4.11 - rows * 1.29)
    grain += 0.15 * np.sin(cols * 9.17 + rows * 7.31)

    sinkage = np.clip(-heightmap / 0.055, 0.0, 1.0)
    dry = np.array([147.0, 103.0, 57.0])
    compacted = np.array([78.0, 48.0, 27.0])
    colors = dry[None, None, :] * (1.0 - sinkage[..., None])
    colors += compacted[None, None, :] * sinkage[..., None]
    colors *= 1.0 + 0.12 * grain[..., None]
    return np.clip(colors, 0.0, 255.0).astype(np.uint8).reshape(-1, 3)


def rolling_terrain_colors(heightmap: np.ndarray) -> np.ndarray:
    low = np.array([68.0, 88.0, 64.0])
    middle = np.array([112.0, 105.0, 72.0])
    high = np.array([151.0, 119.0, 68.0])
    scale = np.clip((heightmap + 0.07) / 0.15, 0.0, 1.0)
    lower_mix = np.minimum(scale * 2.0, 1.0)[..., None]
    upper_mix = np.maximum(scale * 2.0 - 1.0, 0.0)[..., None]
    colors = low * (1.0 - lower_mix) + middle * lower_mix
    colors = colors * (1.0 - upper_mix) + high * upper_mix
    rows, cols = np.indices(heightmap.shape)
    grain = 1.0 + 0.035 * np.sin(cols * 2.7 + rows * 1.9)[..., None]
    return np.clip(colors * grain, 0.0, 255.0).astype(np.uint8).reshape(-1, 3)


def world_robot_mesh(body, robot_cfg: dict, joint_positions=None):
    if joint_positions is None:
        vertices, faces, face_colors = load_go1_visual(robot_cfg)
    else:
        vertices, faces, face_colors = load_go1_articulated_visual(robot_cfg, joint_positions)
    pos = body.GetPos()
    rot = body.GetRot()
    w, x, y, z = float(rot.e0), float(rot.e1), float(rot.e2), float(rot.e3)
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )
    world_vertices = vertices @ rotation.T + np.array([pos.x, pos.y, pos.z])
    return world_vertices, faces, face_colors[:, :3]


def pyvista_runtime_diagnostic() -> str | None:
    """Return an actionable error before VTK enters a broken headless path."""
    if os.name != "posix" or os.environ.get("DISPLAY"):
        return None
    if os.environ.get("PYVISTA_TRAME_SERVER_PROXY_PREFIX"):
        return None
    try:
        import vtk

        render_window = vtk.vtkRenderWindow()
        class_name = render_window.GetClassName()
    except Exception as exc:  # pragma: no cover - import failures vary by wheel
        return f"VTK render-window initialization failed: {exc}"
    if class_name == "vtkXOpenGLRenderWindow":
        return "VTK requires an X display. Run under a desktop DISPLAY or install/use Xvfb."
    return None


@dataclass
class FrameContext:
    heightmap: np.ndarray
    initial_heightmap: np.ndarray
    robot_cfg: dict
    body: object
    sim_time: float
    selected_text: str
    gait_state: object | None
    contact_feet: object | None
    traversal: bool
    reveal_alpha: float
    hazard_triggered: bool = False
    hazard_strike_leg: str | None = None
    hazard_slip_distance_m: float = 0.0
    maneuver_phase: str | None = None


class PyVistaFrameRenderer:
    """Persistent VTK scene for Chrono SCM video frames."""

    def __init__(
        self,
        initial_heightmap: np.ndarray,
        robot_cfg: dict,
        *,
        width: int = 1280,
        height: int = 720,
        dem_panel: bool = False,
        dem_max_mm: float = 40.0,
        hazard: RigidHazard | None = None,
        difficult_course: DifficultCourse | None = None,
        rolling_course: RollingCourse | None = None,
        side_view: bool = False,
    ) -> None:
        diagnostic = pyvista_runtime_diagnostic()
        if diagnostic:
            raise RuntimeError(f"PyVista renderer unavailable: {diagnostic}")

        import pyvista as pv

        self.pv = pv
        self.robot_cfg = robot_cfg
        self.dem_panel = dem_panel
        self.dem_max_mm = dem_max_mm
        self.width = width
        self.height = height
        self.hazard = hazard
        self.difficult_course = difficult_course
        self.rolling_course = rolling_course
        self.support_course = difficult_course or rolling_course
        self.side_view = side_view
        shape = (1, 2) if dem_panel else (1, 1)
        self.plotter = pv.Plotter(
            shape=shape,
            off_screen=True,
            window_size=(width, height),
            border=False,
        )
        self.plotter.set_background("#dce2e5", top="#f7f8f7", all_renderers=True)
        self._build_scene(initial_heightmap)

    def _build_scene(self, initial_heightmap: np.ndarray) -> None:
        pv = self.pv
        self.plotter.subplot(0, 0)
        points, faces = surface_mesh_arrays(initial_heightmap)
        self.terrain = pv.PolyData(points, _vtk_faces(faces))
        self.terrain.point_data["soil_rgb"] = self._terrain_colors(initial_heightmap)
        self.terrain_actor = self.plotter.add_mesh(
            self.terrain,
            scalars="soil_rgb",
            rgb=True,
            smooth_shading=True,
            pbr=True,
            ambient=0.24,
            diffuse=0.82,
            specular=0.12,
            specular_power=18.0,
            roughness=0.88,
        )

        for center, size, color in (
            ((-1.05, 0.0, -0.041), (0.9, 3.0, 0.08), "#6f7476"),
            ((1.05, 0.0, -0.041), (0.9, 3.0, 0.08), "#6f7476"),
            ((0.0, -1.05, -0.041), (1.2, 0.9, 0.08), "#6f7476"),
            ((0.0, 1.05, -0.041), (1.2, 0.9, 0.08), "#6f7476"),
            ((0.0, 1.38, 0.22), (0.6, 0.04, 0.36), "#e4df22"),
        ):
            bounds = (
                center[0] - size[0] / 2,
                center[0] + size[0] / 2,
                center[1] - size[1] / 2,
                center[1] + size[1] / 2,
                center[2] - size[2] / 2,
                center[2] + size[2] / 2,
            )
            self.plotter.add_mesh(
                pv.Box(bounds=bounds),
                color=color,
                smooth_shading=False,
                pbr=True,
                ambient=0.22,
                diffuse=0.76,
                specular=0.16,
                roughness=0.72,
            )

        if self.hazard is not None:
            hazard = self.hazard
            self.plotter.add_mesh(
                pv.Box(
                    bounds=(
                        hazard.center_x_m - hazard.size_x_m / 2,
                        hazard.center_x_m + hazard.size_x_m / 2,
                        hazard.center_y_m - hazard.size_y_m / 2,
                        hazard.center_y_m + hazard.size_y_m / 2,
                        0.0,
                        hazard.height_m,
                    )
                ),
                color="#d33a13",
                pbr=True,
                roughness=0.58,
                metallic=0.08,
            )
        if self.difficult_course is not None:
            colors = ("#dc741b", "#2e7ca7", "#b7a023")
            for pad, color in zip(self.difficult_course.pads, colors, strict=True):
                self.plotter.add_mesh(
                    pv.Box(
                        bounds=(
                            pad.center_x_m - pad.size_x_m / 2,
                            pad.center_x_m + pad.size_x_m / 2,
                            pad.center_y_m - pad.size_y_m / 2,
                            pad.center_y_m + pad.size_y_m / 2,
                            0.0,
                            pad.height_m,
                        )
                    ),
                    color=color,
                    pbr=True,
                    roughness=0.72,
                    metallic=0.04,
                )

        self.robot_mesh = None
        self.robot_actor = None
        self.hud = self.plotter.add_text(
            "",
            position=(18, max(self.height - (214 if self.hazard is not None or self.support_course is not None else 170), 12)),
            font_size=10,
            color="#171b1d",
            shadow=False,
        )
        self.hud.GetTextProperty().SetBackgroundColor(0.94, 0.95, 0.95)
        self.hud.GetTextProperty().SetBackgroundOpacity(0.72)
        if self.side_view:
            self.plotter.camera.position = (2.85, -0.05, 0.72)
            self.plotter.camera.focal_point = (0.0, -0.05, 0.16)
            self.plotter.camera.view_angle = 35.0
            self.plotter.camera.zoom(1.12)
        else:
            self.plotter.camera.position = (2.65, -3.20, 2.35)
            self.plotter.camera.focal_point = (0.0, -0.02, 0.10)
            self.plotter.camera.view_angle = 40.0
        self.plotter.camera.up = (0.0, 0.0, 1.0)
        self.plotter.remove_all_lights()
        self.plotter.add_light(pv.Light(position=(-2.8, -3.4, 4.8), focal_point=(0, 0, 0), intensity=0.92))
        self.plotter.add_light(pv.Light(position=(2.0, 1.2, 2.6), focal_point=(0, 0, 0), intensity=0.35))
        try:
            self.plotter.enable_shadows()
            self.plotter.enable_anti_aliasing("fxaa")
        except RuntimeError:
            pass

        if self.dem_panel:
            self.plotter.subplot(0, 1)
            flat = np.zeros_like(initial_heightmap)
            points, faces = surface_mesh_arrays(flat)
            self.dem_mesh = pv.PolyData(points, _vtk_faces(faces))
            self.dem_mesh.point_data["difference_mm"] = np.zeros(points.shape[0])
            self.plotter.add_mesh(
                self.dem_mesh,
                scalars="difference_mm",
                cmap="coolwarm_r",
                clim=(-self.dem_max_mm, self.dem_max_mm),
                lighting=False,
                nan_color="#d8d8d8",
                scalar_bar_args={
                    "title": "Elevation change (mm)",
                    "vertical": False,
                    "position_x": 0.16,
                    "position_y": 0.04,
                    "width": 0.68,
                    "height": 0.08,
                    "title_font_size": 13,
                    "label_font_size": 11,
                },
            )
            self.dem_title = self.plotter.add_text(
                "DEM difference\ncurrent surface - initial surface",
                position="upper_edge",
                font_size=11,
                color="#171b1d",
            )
            self.dem_stats = self.plotter.add_text("", position=(18, 108), font_size=9, color="#171b1d")
            self.foot_points = pv.PolyData(np.zeros((4, 3)))
            self.plotter.add_mesh(
                self.foot_points,
                color="#111111",
                point_size=13,
                render_points_as_spheres=True,
                lighting=False,
            )
            self.plotter.camera_position = "xy"
            self.plotter.camera.parallel_projection = True
            self.plotter.camera.parallel_scale = 0.72
            self.plotter.set_background("#eef1f2")
            if self.hazard is not None:
                hazard = self.hazard
                self.plotter.add_mesh(
                    pv.Box(
                        bounds=(
                            hazard.center_x_m - hazard.size_x_m / 2,
                            hazard.center_x_m + hazard.size_x_m / 2,
                            hazard.center_y_m - hazard.size_y_m / 2,
                            hazard.center_y_m + hazard.size_y_m / 2,
                            0.004,
                            0.008,
                        )
                    ),
                    color="#d33a13",
                    opacity=0.55,
                    lighting=False,
                )

    def _update_robot(self, body, gait_state, opacity: float) -> None:
        robot = self.robot_cfg["robot"]
        if robot.get("visual_asset") != "go1_urdf":
            raise ValueError("The PyVista backend currently supports the Go1 visual asset")
        joints = gait_state.joint_positions if gait_state is not None else None
        vertices, faces, colors = world_robot_mesh(body, self.robot_cfg, joints)
        if self.robot_mesh is None:
            self.robot_mesh = self.pv.PolyData(vertices, _vtk_faces(faces))
            self.robot_mesh.cell_data["robot_rgb"] = colors
            self.robot_actor = self.plotter.add_mesh(
                self.robot_mesh,
                scalars="robot_rgb",
                rgb=True,
                smooth_shading=True,
                pbr=True,
                ambient=0.22,
                diffuse=0.82,
                specular=0.36,
                specular_power=32.0,
                roughness=0.48,
            )
        else:
            self.robot_mesh.points = vertices
            self.robot_mesh.cell_data["robot_rgb"] = colors
            self.robot_mesh.Modified()
        self.robot_actor.GetProperty().SetOpacity(opacity)

    def _terrain_colors(self, heightmap: np.ndarray) -> np.ndarray:
        if self.rolling_course is not None:
            return rolling_terrain_colors(heightmap)
        if self.hazard is not None or self.difficult_course is not None:
            return np.tile(np.array([100, 105, 107], dtype=np.uint8), (heightmap.size, 1))
        return soil_point_colors(heightmap)

    def _foot_positions(self, context: FrameContext) -> np.ndarray:
        if self.support_course is not None and context.gait_state is not None:
            body_pos = context.body.GetPos()
            body_rot = context.body.GetRot()
            positions = [
                body_pos + body_rot.Rotate(self._chrono_vector(offset))
                for offset in context.gait_state.foot_positions_body.values()
            ]
        elif context.contact_feet is not None:
            positions = [foot.GetPos() for foot in context.contact_feet.bodies.values()]
        else:
            body_pos = context.body.GetPos()
            body_rot = context.body.GetRot()
            offsets = foot_offsets(context.robot_cfg)
            positions = [body_pos + body_rot.Rotate(self._chrono_vector(offset)) for offset in offsets]
        return np.array([[pos.x, pos.y, 0.006] for pos in positions])

    @staticmethod
    def _chrono_vector(values):
        return chrono.ChVector3d(*values)

    def render(self, context: FrameContext) -> np.ndarray:
        self.plotter.subplot(0, 0)
        display_heightmap = context.heightmap.copy()
        vertical_exaggeration = 1.0 + 2.0 * context.reveal_alpha
        display_heightmap[1:-1, 1:-1] *= vertical_exaggeration
        points, _ = surface_mesh_arrays(display_heightmap)
        self.terrain.points = points
        self.terrain.point_data["soil_rgb"] = self._terrain_colors(display_heightmap)
        self.terrain.Modified()
        self._update_robot(context.body, context.gait_state, 1.0 - 0.72 * context.reveal_alpha)

        if context.maneuver_phase is not None:
            motion_name = "forward-turn-forward"
        else:
            motion_name = "velocity-command trot" if context.traversal else "scripted approach"
        if self.hazard is not None:
            terrain_name = "rigid hazard course"
        elif self.rolling_course is not None:
            terrain_name = "rolling SCM hills + valleys"
        elif self.difficult_course is not None:
            terrain_name = "rigid difficult course"
        else:
            terrain_name = "Chrono SCM soil"
        selected_text = context.selected_text.replace("; ", "\n", 1)
        hud = (
            f"{context.robot_cfg['robot']['name'].upper()} {motion_name} + {terrain_name}\n"
            f"{selected_text}\n"
            f"t = {context.sim_time:0.2f} s"
        )
        if self.hazard is not None or self.difficult_course is not None:
            hud += "\nRigid course deformation: none"
        elif self.rolling_course is not None:
            mean_sinkage, max_sinkage, active_nodes = self._difference_stats(
                context.initial_heightmap, context.heightmap
            )
            hud += (
                f"\nSCM displaced nodes: mean {mean_sinkage * 1000:0.1f} mm"
                f"\nmax {max_sinkage * 1000:0.1f} mm; nodes {active_nodes}"
            )
        else:
            mean_sinkage, max_sinkage, _ = self._deformation_stats(context.heightmap)
            hud += (
                f"\nSCM displaced nodes: mean {mean_sinkage * 1000:0.1f} mm"
                f"\nmax {max_sinkage * 1000:0.1f} mm"
            )
        if context.traversal:
            if context.maneuver_phase is not None:
                hud += f"\nmaneuver phase: {context.maneuver_phase}"
            if self.hazard is not None:
                hud += "\nOpen-loop gait; geometric toe-strike trigger"
            elif self.support_course is not None:
                up = context.body.GetRot().Rotate(chrono.ChVector3d(0.0, 0.0, 1.0))
                tilt_deg = np.degrees(np.arccos(np.clip(float(up.z), -1.0, 1.0)))
                hud += (
                    "\nOpen-loop gait; trunk follows fitted support plane"
                    f"\ncurrent trunk tilt {tilt_deg:0.1f} deg"
                    "\nKinematic terrain following; no balance controller"
                )
                if self.side_view and context.gait_state is not None:
                    local = context.gait_state.foot_positions_body["FR"]
                    world = context.body.GetPos() + context.body.GetRot().Rotate(
                        chrono.ChVector3d(*local)
                    )
                    foot_half_height = 0.5 * float(
                        context.robot_cfg["robot"]["foot_height_m"]
                    )
                    ground = self.support_course.height_at(float(world.x), float(world.y))
                    clearance_mm = (float(world.z) - foot_half_height - ground) * 1000.0
                    phase_name = "stance" if context.gait_state.stance["FR"] else "swing"
                    hud += f"\nFR foot-bottom clearance {clearance_mm:0.1f} mm ({phase_name})"
            else:
                hud += "\nOpen-loop gait; independent stance feet drive SCM contact"
        if self.hazard is not None:
            if context.hazard_triggered:
                up = context.body.GetRot().Rotate(chrono.ChVector3d(0.0, 0.0, 1.0))
                tilt_deg = np.degrees(np.arccos(np.clip(float(up.z), -1.0, 1.0)))
                hud += (
                    f"\nTOE CONTACT ({context.hazard_strike_leg}): one-side support lost"
                    f"\nlateral skid {context.hazard_slip_distance_m:0.2f} m"
                    f"\ntrunk tilt {tilt_deg:0.1f} deg"
                    "\nReduced-order skid + rigid-body fall; no controller"
                )
            else:
                hud += "\nRIGID OFFSET SLIP HAZARD: awaiting foot contact"
        if context.reveal_alpha > 0.05:
            hud += f"\nFootprint reveal: {vertical_exaggeration:0.1f}x vertical exaggeration"
        self.hud.SetInput(hud)

        if self.dem_panel:
            self.plotter.subplot(0, 1)
            difference = (context.heightmap - context.initial_heightmap).astype(float) * 1000.0
            difference[[0, -1], :] = np.nan
            difference[:, [0, -1]] = np.nan
            self.dem_mesh.point_data["difference_mm"] = difference.ravel()
            finite = np.isfinite(difference)
            subsidence = -difference[finite & (difference < -0.01)]
            uplift = difference[finite & (difference > 0.01)]
            mean_sub = float(np.mean(subsidence)) if subsidence.size else 0.0
            max_sub = float(np.max(subsidence)) if subsidence.size else 0.0
            max_up = float(np.max(uplift)) if uplift.size else 0.0
            self.dem_stats.SetInput(
                f"mean subsidence: {mean_sub:0.1f} mm\n"
                f"max subsidence: {max_sub:0.1f} mm\n"
                f"max uplift: {max_up:0.1f} mm"
            )
            self.dem_mesh.Modified()
            self.foot_points.points = self._foot_positions(context)
            self.foot_points.Modified()

        self.plotter.render()
        return np.asarray(self.plotter.screenshot(return_img=True))[:, :, :3].copy()

    @staticmethod
    def _deformation_stats(heightmap: np.ndarray) -> tuple[float, float, int]:
        interior = np.maximum(-heightmap[1:-1, 1:-1], 0.0)
        active = interior[interior > 1.0e-5]
        if active.size == 0:
            return 0.0, 0.0, 0
        return float(np.mean(active)), float(np.max(active)), int(active.size)

    @staticmethod
    def _difference_stats(
        initial_heightmap: np.ndarray,
        heightmap: np.ndarray,
    ) -> tuple[float, float, int]:
        sinkage = initial_heightmap[1:-1, 1:-1] - heightmap[1:-1, 1:-1]
        active = sinkage[sinkage > 1.0e-5]
        if active.size == 0:
            return 0.0, 0.0, 0
        return float(np.mean(active)), float(np.max(active)), int(active.size)

    def close(self) -> None:
        self.plotter.close()
