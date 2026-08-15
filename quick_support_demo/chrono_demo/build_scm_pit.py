from __future__ import annotations

import numpy as np

from .chrono_import import import_chrono
from .difficult_terrain import RollingCourse, build_rolling_mesh


chrono, veh = import_chrono()


def build_scm_pit(system: object, terrain_cfg: dict, visualization_mesh: bool = True) -> object:
    pit_cfg = terrain_cfg["pit"]
    soil_cfg = terrain_cfg["soil"]
    terrain = veh.SCMTerrain(system, visualization_mesh)
    terrain.SetPlane(
        chrono.ChCoordsysd(
            chrono.ChVector3d(0.0, 0.0, float(pit_cfg["top_elevation_m"])),
            chrono.QUNIT,
        )
    )
    terrain.Initialize(
        float(pit_cfg["size_m"][0]),
        float(pit_cfg["size_m"][1]),
        float(pit_cfg["grid_spacing_m"]),
    )
    terrain.SetSoilParameters(
        float(soil_cfg["bekker_kphi"]),
        float(soil_cfg["bekker_kc"]),
        float(soil_cfg["bekker_n"]),
        float(soil_cfg["mohr_cohesion"]),
        float(soil_cfg["mohr_friction_deg"]),
        float(soil_cfg["janosi_shear_m"]),
        float(soil_cfg["elastic_k"]),
        float(soil_cfg["damping_r"]),
    )
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.06)
    return terrain


def build_rolling_scm_pit(
    system: object,
    terrain_cfg: dict,
    course: RollingCourse,
    visualization_mesh: bool = True,
) -> object:
    pit_cfg = terrain_cfg["pit"]
    soil_cfg = terrain_cfg["soil"]
    initial_heightmap = course.heightmap(terrain_cfg)
    # Mesh-initialized SCM offsets the source mesh by its lower relief extent.
    # Compensate so the course's tapered zero-height boundary stays at floor Z.
    mesh_reference_height = -float(np.min(initial_heightmap))
    terrain = veh.SCMTerrain(system, visualization_mesh)
    terrain.SetPlane(
        chrono.ChCoordsysd(
            chrono.ChVector3d(
                0.0,
                0.0,
                float(pit_cfg["top_elevation_m"]) + mesh_reference_height,
            ),
            chrono.QUNIT,
        )
    )
    terrain.Initialize(
        build_rolling_mesh(course, terrain_cfg),
        float(pit_cfg["grid_spacing_m"]),
    )
    terrain.SetSoilParameters(
        float(soil_cfg["bekker_kphi"]),
        float(soil_cfg["bekker_kc"]),
        float(soil_cfg["bekker_n"]),
        float(soil_cfg["mohr_cohesion"]),
        float(soil_cfg["mohr_friction_deg"]),
        float(soil_cfg["janosi_shear_m"]),
        float(soil_cfg["elastic_k"]),
        float(soil_cfg["damping_r"]),
    )
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.06)
    return terrain


class SCMHeightCourse:
    def __init__(
        self,
        terrain: object,
        size_m: tuple[float, float],
        outside_height_m: float = 0.0,
    ) -> None:
        self.terrain = terrain
        self.half_size_x_m = 0.5 * float(size_m[0])
        self.half_size_y_m = 0.5 * float(size_m[1])
        self.outside_height_m = float(outside_height_m)

    def height_at(self, x_m: float, y_m: float) -> float:
        if abs(x_m) > self.half_size_x_m or abs(y_m) > self.half_size_y_m:
            return self.outside_height_m
        return float(self.terrain.GetHeight(chrono.ChVector3d(x_m, y_m, 0.0)))


def sample_heightmap(terrain: object, terrain_cfg: dict) -> np.ndarray:
    size_x, size_y = terrain_cfg["pit"]["size_m"]
    spacing = float(terrain_cfg["pit"]["grid_spacing_m"])
    xs = np.arange(-0.5 * size_x, 0.5 * size_x + 0.5 * spacing, spacing)
    ys = np.arange(-0.5 * size_y, 0.5 * size_y + 0.5 * spacing, spacing)
    heightmap = np.zeros((len(ys), len(xs)), dtype=np.float32)
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            heightmap[iy, ix] = terrain.GetHeight(chrono.ChVector3d(float(x), float(y), 0.0))
    if not np.isfinite(heightmap).all():
        raise RuntimeError("SCM heightmap contains non-finite values")
    expected_shape = (len(ys), len(xs))
    if heightmap.shape != expected_shape:
        raise RuntimeError(f"SCM heightmap shape {heightmap.shape} does not match {expected_shape}")
    return heightmap
