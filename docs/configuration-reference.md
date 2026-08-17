# Configuration Reference

[Documentation index](README.md)

## Configuration composition

`quick_support_demo/configs/demo.yaml` names the world, terrain, candidate, and
robot files. `load_demo_config()` loads them into:

```text
cfg["world"]
cfg["terrain"]
cfg["candidates"]
cfg["robots"]["go1"]
cfg["robots"]["spot"]
```

All dimensions are SI unless a CLI flag explicitly says millimeters.

## World

Source: `quick_support_demo/configs/world.yaml`

| Key | Current value | Meaning |
|---|---:|---|
| `world.floor_size_m` | `[3.0, 3.0]` | Overall floor extent |
| `world.gravity_mps2` | `[0, 0, -9.81]` | Gravity vector |
| `world.timestep_s` | `0.0005` | Configured physics time step |
| `world.settle_time_s` | `3.0` | Maximum stationary loading time |
| `world.settle_velocity_mps` | `0.005` | Linear settled threshold |
| `world.settle_ang_velocity_radps` | `0.01` | Angular settled threshold |
| `floor.thickness_m` | `0.08` | Rigid floor-box thickness |
| `floor.density_kgpm3` | `1000` | Floor body density |
| `floor.friction` | `0.8` | Rigid floor contact friction |

## Terrain

Source: `quick_support_demo/configs/terrain.yaml`

| Key | Current value | Meaning |
|---|---:|---|
| `pit.model` | `SCM` | Chrono soil contact model |
| `pit.size_m` | `[1.2, 1.2]` | SCM patch extent |
| `pit.grid_spacing_m` | `0.01` | Configured SCM node spacing |
| `pit.top_elevation_m` | `0.0` | Initial terrain surface elevation |
| `soil.bekker_kphi` | `800000` | Frictional pressure-sinkage modulus |
| `soil.bekker_kc` | `25000` | Cohesive pressure-sinkage modulus |
| `soil.bekker_n` | `1.1` | Pressure-sinkage exponent |
| `soil.mohr_cohesion` | `1000` | Mohr-Coulomb cohesion input |
| `soil.mohr_friction_deg` | `28` | Internal friction angle in degrees |
| `soil.janosi_shear_m` | `0.01` | Janosi-Hanamoto shear length |
| `soil.elastic_k` | `20000000` | Elastic stiffness input |
| `soil.damping_r` | `30000` | Damping input |

These values were selected for visible deformation and are not calibrated
measurements of a named real sand.

## Go1

Source: `quick_support_demo/configs/go1.yaml`

| Key | Value |
|---|---:|
| model | `urdf_visual_proxy_contact` |
| visual asset | `go1_urdf` |
| mass | `12.5 kg` |
| payload | `0 kg` |
| stance length | `0.3762 m` |
| stance width | `0.2535 m` |
| foot radius | `0.035 m` |
| foot height | `0.035 m` |
| proxy body size | `[0.62, 0.25, 0.16] m` |
| COM height | `0.30 m` |
| start clearance | `0.005 m` |

## Spot

Source: `quick_support_demo/configs/spot.yaml`

| Key | Value |
|---|---:|
| model | `proxy` |
| mass | `32.5 kg` |
| payload | `0 kg` |
| stance length | `0.70 m` |
| stance width | `0.36 m` |
| foot radius | `0.032 m` |
| foot height | `0.04 m` |
| proxy body size | `[0.84, 0.32, 0.20] m` |
| COM height | `0.38 m` |
| start clearance | `0.005 m` |

Spot has no detailed visual asset in the current repository.

## Candidates and planning

Source: `quick_support_demo/configs/candidates.yaml`

| Candidate | Base `(x, y)` | View cost | Path cost |
|---|---:|---:|---:|
| sand | `(0.0, -0.05) m` | `0.0` | `0.0` |
| rigid | `(-1.05, -0.20) m` | `0.50` | `0.50` |

Planning weights:

| Term | Weight |
|---|---:|
| view | `1.0` |
| path | `1.0` |
| maximum sinkage | `45.0` |
| maximum absolute tilt | `3.0` |
| uncertainty | `0.0` |

## Smoke overrides

Smoke mode is deliberately explicit and differs by entry point.

`run_demo.py --smoke` applies:

- time step `0.001 s`;
- maximum settle time `0.6 s`;
- SCM grid spacing `0.04 m`.

`make_chrono_3d_video --smoke` applies:

- time step `0.001 s`;
- SCM grid spacing `0.035 m`;
- no change to video duration unless supplied on the CLI.

Without `--smoke`, both workflows use the configured `10 mm` SCM grid.

## Video CLI

Important flags on `make_chrono_3d_video`:

| Flag | Default | Meaning |
|---|---:|---|
| `--robot` | `go1` | `go1` or `spot` |
| `--duration` | `6.0` | Simulated video duration in seconds |
| `--fps` | `12` | Captured frames per second |
| `--mass-scale` | `1.0` | Multiplies robot and payload mass |
| `--dem-panel` | off | Adds synchronized `current - initial` DEM viewport for SCM |
| `--dem-max-mm` | `40` | SCM difference DEM magnitude/color limit |
| `--traverse` | off | Enables velocity-command traversal |
| `--vx` | `0.25` | Body-frame forward speed, m/s |
| `--vy` | `0.0` | Body-frame lateral speed, m/s |
| `--wz` | `0.0` | Body yaw rate, rad/s |
| `--gait-frequency` | `1.6` | Open-loop trot frequency, Hz |
| `--step-height` | `0.055` | Swing-foot clearance, m |
| `--renderer` | `matplotlib` | `matplotlib` or `pyvista` |
| `--width` | `1280` | Output width |
| `--height` | `720` | Output height |
| `--hazard` | off | Use rigid center plate, offset trip block, and reduced-order fall |
| `--hazard-offset-x` | `0.13` | Block center lateral x offset, m |
| `--hazard-height` | `0.13` | Block height, m |
| `--hazard-slip-speed` | `0.55` | Prescribed lateral release speed, m/s |
| `--hazard-tip-rate` | `0.65` | Initial tipping angular speed, rad/s |
| `--difficult-terrain` | off | Traverse three staggered nondeforming pads |
| `--rolling-terrain` | off | Traverse deformable SCM initialized with a smooth hills-and-valleys mesh |
| `--forward-turn-forward` | off | Replace the constant command with forward, in-place turn, forward phases |
| `--first-forward-distance` | `0.85` | First maneuver segment distance, m |
| `--turn-angle-deg` | `-90` | Signed turn angle; negative is right, positive is left |
| `--turn-rate` | `0.8` | Absolute in-place turn rate, rad/s |
| `--second-forward-distance` | `0.90` | Second maneuver segment distance, m |
| `--difficult-max-tilt-deg` | `14` | Per-axis support-plane tilt limit, deg |
| `--side-view` | off | Lateral camera for foot height and touchdown inspection |
| `--splat-output` | none | New directory for calibrated final-state RGB-D orbit views |
| `--splat-hide-robot` | off | Exclude robot geometry from orbit RGB and depth |
| `--orbit-theta-deg` | `15,30,45,60` | Comma-separated elevation rings |
| `--orbit-phi-count` | `36` | Uniform 360-degree azimuth samples per ring |
| `--orbit-phi-deg` | none | Explicit azimuth list overriding uniform sampling |
| `--orbit-phi-offset-deg` | `0` | Offset for uniform azimuth samples |
| `--orbit-radius` | `3.2` | Camera radius in meters |
| `--orbit-target` | `0,0,0.15` | World-space look-at point in meters |
| `--orbit-view-angle-deg` | `45` | Vertical field of view |
| `--smoke` | off | Coarse 35 mm video grid |
| `--full-res` | default | Configured 10 mm grid |

Traversal, hazard mode, difficult-terrain mode, rolling-terrain mode, and the PyVista backend
currently require Go1. Both rigid modes require `--traverse`, do not create
SCM, and make smoke/full-resolution grid selection physically irrelevant.
`--hazard`, `--difficult-terrain`, and `--rolling-terrain` are mutually
exclusive. Rolling terrain requires `--traverse`; unlike the rigid-pad mode, it
supports `--dem-panel` because it owns a deformable SCM surface.
Forward-turn-forward also requires `--traverse`, requires positive `--vx`, and
is intentionally rejected with hazard mode.
Orbit capture requires `--renderer pyvista`, samples the final state at
`--duration`, and writes RGB PNG, metric float32 depth, uint16 millimeter depth,
and OpenGL camera-to-world poses. See
[Splat RGB-D capture](splat-rgbd-capture.md) for the full schema.

## Parameter-change discipline

When changing soil or robot parameters:

1. Save the full configuration with the artifact.
2. Do not compare values from different grid spacings as if they are one run.
3. Keep `mass_scale=1` for physically nominal claims.
4. Label smoke versus full-resolution outputs.
5. Re-run the same baseline before comparing a renderer or model change.
