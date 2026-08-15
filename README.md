# Chronos Quick Support Demo

PyChrono scaffold for the quick visual demo in `QUICK_VISUAL_DEMO_PLAN.md`.

The canonical, structured documentation starts at
[`docs/README.md`](docs/README.md). It links specialist references for the
architecture, configuration, physics, robot/gait model, rendering, planning,
operations, validation, limitations, and roadmap.

For the full implementation inventory, measured results, known correctness
issues, and recommended next steps, see `CURRENT_SYSTEM_STATE.md`.

The current implementation covers the first support-planning milestone:

- rigid perimeter floor with a central Chrono SCM terrain patch;
- official Go1 visual geometry in a fixed home stance, with hidden simplified
  foot contacts for SCM physics;
- four-foot locked-stance proxy for Spot;
- two candidate poses: close sand and rigid bypass;
- shared support-aware objective;
- JSON summaries and NumPy height maps for each trial.

## Environment

Use the existing conda environment:

```bash
conda activate chrono_splat
```

The required packages were installed with:

```bash
conda install -n chrono_splat -c conda-forge numpy pyyaml opencv matplotlib imageio imageio-ffmpeg
conda install -n chrono_splat -c conda-forge trimesh pycollada fast-simplification
conda install -n chrono_splat -c projectchrono -c conda-forge pychrono=8.0.0
conda run -n chrono_splat pip install pyvista vtk
```

The ProjectChrono channel build is required because the conda-forge PyChrono 10 build available here exposes `core`, `fea`, and `robot`, but not `pychrono.vehicle.SCMTerrain`.

## Run

Fast validation:

```bash
conda run -n chrono_splat python run_demo.py --robot all --candidate all --smoke
```

Single-trial debugging:

```bash
conda run -n chrono_splat python run_demo.py --robot go1 --candidate sand --smoke
```

Higher-fidelity configured run:

```bash
conda run -n chrono_splat python run_demo.py --robot all --candidate all
```

Outputs are written under `quick_support_demo/outputs/trials/<timestamp>/`.

## Preview Video

Create a quick 2D preview video from the latest trial outputs:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos conda run -n chrono_splat python -m quick_support_demo.overlays.make_preview_video
```

The preview is not a Chrono 3D render. It visualizes the measured SCM heightmap deformation and the Go1/Spot pose decisions while the full visualizer capture is still pending.

Create a 3D video from live Chrono SCM simulation states:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video --robot go1 --duration 6.0 --fps 12
```

This uses Chrono for contact/deformation and Matplotlib for deterministic frame
rendering. Go1 is drawn from the included Unitree URDF and BSD-licensed
simplified link meshes. The links are frozen in the model's home stance and
follow the simulated base as one body in this stationary-load mode. The
velocity-command mode below adds articulated visual gait motion, but there is
still no articulated Chrono contact model or closed-loop controller.

The checked output generated during development is:

```text
quick_support_demo/outputs/videos/go1_chrono_dirt_visual.mp4
```

For a deliberately heavy visual variant with more visible physical sinkage:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video --robot go1 --mass-scale 4 --duration 6.0 --fps 12
```

The mass multiplier changes the Chrono body mass and inertia through the normal
proxy construction. During the final footprint-reveal shot only, the renderer
fades the robot mesh and smoothly increases terrain vertical exaggeration to
3x. The on-screen millimeter measurements remain the unscaled Chrono values.

Create the corrected nominal-mass synchronized loading and DEM-difference
video:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video --robot go1 --duration 6.0 --fps 6 --dem-panel --dem-max-mm 12 --smoke --output quick_support_demo/outputs/videos/go1_loading_dem_nominal_12p5kg.mp4
```

The left panel shows the live Chrono loading scene at true vertical scale. The
right panel shows signed elevation change, `(current surface - initial
surface)`, from the same timestep. Red is subsidence, blue is uplift, and the
outer SCM boundary ring is masked because its base-level offset is not robot
deformation. `--dem-max-mm` fixes the subsidence end of the color scale in
millimeters so colors remain comparable over the full video.

The corrected renderer defaults to the configured `10 mm` SCM grid. `--smoke`
is explicit and uses a `35 mm` grid for practical video generation. The command
above uses the configured `12.5 kg` Go1 mass and reports the coarse grid in the
video itself. Use `--full-res` for the configured grid when runtime permits.

## Velocity-Command Motion

Generate a straight traversal from the near floor, across the pit, to the far
floor:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video --robot go1 --traverse --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.055 --duration 11 --fps 6 --dem-panel --dem-max-mm 30 --smoke --output quick_support_demo/outputs/videos/go1_velocity_trot_traverse_independent_feet_12p5kg.mp4
```

`--vx`, `--vy`, and `--wz` are forward, lateral, and yaw-rate commands in the
robot body frame. `TrotGait` converts them into phase-offset stance and swing
foot trajectories, solves three-joint inverse kinematics for each leg, and
animates all 12 Go1 joints. Four independent Chrono foot bodies carry the full
`12.5 kg` load across active stance contacts. Swing feet disable collision and
follow the commanded clearance arc; stance feet remain approximately planted,
settle dynamically in `z`, and produce discrete SCM footprints. This is an
open-loop contact approximation, not a ROS 2 controller or dynamically balanced
multibody gait: the trunk is kinematic and the feet are not connected to it by
physical leg joints.

## PyVista / VTK Renderer

Use `--renderer pyvista` to render the same live Chrono simulation with VTK:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.055 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 30 --smoke \
  --output quick_support_demo/outputs/videos/go1_velocity_trot_pyvista_dem_12p5kg.mp4
```

The PyVista backend uses a persistent VTK scene with the articulated Go1 mesh,
a dynamically updated SCM surface, granular soil albedo, PBR materials,
multiple lights, shadows, antialiasing, and an optional native VTK DEM-difference
viewport. `--renderer matplotlib` remains the default and preserves the prior
deterministic rendering path. `--width` and `--height` control video resolution.

VTK requires a working OpenGL render window. The installed Linux wheel can use
EGL on this workstation when given graphics-device access. On a headless system
without EGL or OSMesa, run under Xvfb or install an EGL/OSMesa-enabled VTK build.

## Rigid Offset Hazard

Generate a nondeforming obstacle showcase and reduced-order Chrono fall:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --hazard \
  --hazard-offset-x 0.13 --hazard-height 0.13 \
  --hazard-slip-speed 0.55 --hazard-tip-rate 0.65 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.055 \
  --duration 7 --fps 8 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4
```

This mode replaces SCM with a rigid center plate and fixed offset block. A
geometric toe strike releases a low-friction rigid proxy with support retained
only on the opposite side. The prescribed lateral release velocity produces a
visible skid before the Chrono rigid-body fall. It has no articulated controller
or leg-force transmission.

## Rigid Difficult Terrain

Generate a complete uneven-course traversal with visible trunk attitude:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --difficult-terrain \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4
```

Three staggered rigid pads create alternating support heights. The trunk follows
a smoothed fitted support plane, reaches the far floor, and remains upright.
This is kinematic terrain following, not closed-loop balance control.

Add `--side-view` to render a lateral diagnostic. The side-view HUD reports the
front-right foot-bottom clearance and stance/swing state. Terrain-aware visual
IK places stance feet directly on the current rigid surface instead of retaining
the previous pad height.

## Deformable Rolling Terrain With DEM

Generate a traversal over deformable SCM hills and valleys with the 3D scene
and deformation DEM side by side:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain --dem-panel \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-max-mm 140 --smoke \
  --output quick_support_demo/outputs/videos/go1_rolling_hills_valleys_scm_deformation_pyvista_dem_12p5kg.mp4
```

The heightfield initializes Chrono SCM from a triangle mesh. Its initial range
is about `-64.2 mm` to `+76.9 mm`; foot contacts then deform the live surface.
The right panel shows `current - initial` elevation. Outside the SCM bounds,
support-height queries return the rigid-floor elevation so the spawn feet do
not follow the SCM mesh baseline into the perimeter slab.

## Forward, Turn, Forward

Add `--forward-turn-forward` to replace the constant traversal command with
three velocity-command phases:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain \
  --forward-turn-forward --vx 0.25 \
  --first-forward-distance 0.85 --turn-angle-deg -90 --turn-rate 0.8 \
  --second-forward-distance 0.90 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 140 --smoke \
  --output quick_support_demo/outputs/videos/go1_rolling_scm_forward_turn_forward_pyvista_dem_12p5kg.mp4
```

Negative turn angles turn right; positive angles turn left. The verified run
ends at `(0.900, -0.249) m`, yaw `0.0 deg`, with mean `29.02 mm` and maximum
`73.88 mm` subsidence across 137 nodes, and leaves a bent deformation track.
The maneuver is an open-loop command sequence, not path following or feedback
navigation.

## Irrlicht Visualizer

The Irrlicht scene script is:

```bash
conda run -n chrono_splat python -m quick_support_demo.chrono_demo.run_visualizer --robot go1 --smoke
```

Headless Irrlicht recording currently needs Xvfb plus Mesa software OpenGL. In this environment, the window initializes under Xvfb/Mesa, but Chrono's Irrlicht screenshot path returns black frames, so the reliable recorded artifact is the Chrono-driven Matplotlib 3D video above.
