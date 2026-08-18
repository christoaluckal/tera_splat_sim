# Chronos Quick Support Demo Documentation

This directory is the canonical documentation set for the current repository.
It separates implemented behavior from historical results and proposed future
work. Start here instead of treating every root-level Markdown file as current.

## System in one paragraph

The repository uses PyChrono 8 for stationary support trials on a central SCM
soil patch and an open-loop Go1 traversal driven by body velocity commands.
Traversal can use either deformable SCM or a separate rigid hazard course. The
hazard course places an offset rigid block on a rigid center plate, detects a
geometric toe strike, and releases a low-friction locked-leg proxy with only
the opposite-side supports retained. A prescribed lateral velocity creates a
visible skid before Chrono rigid-body dynamics evolve the fall. Videos can be
rendered with Matplotlib or PyVista/VTK, including a
synchronized DEM-difference panel for SCM runs. This is a support and
visualization prototype, not a dynamically balanced articulated quadruped.
It also provides a non-failure difficult-terrain mode: three staggered rigid
pads drive a smoothed support-plane attitude while the robot completes the
crossing. A second non-failure mode initializes Chrono SCM from a continuous
triangle-mesh heightfield, allowing hills and valleys to deform under the
crossing while a synchronized DEM shows `current - initial` elevation.

## Current verified state

| Capability | Status |
|---|---|
| PyChrono rigid world and SCM pit | Implemented |
| Go1 stationary support proxy | Implemented |
| Spot stationary support proxy | Implemented, proxy visual only |
| Go1 URDF-derived visual mesh | Implemented |
| Shared candidate cost and selection | Implemented |
| Body-velocity to open-loop trot | Implemented |
| Forward-turn-forward velocity maneuver | Implemented |
| Independent stance/swing foot contacts | Implemented approximation |
| Matplotlib video backend | Implemented |
| PyVista/VTK PBR video backend | Implemented for Go1 |
| Native VTK DEM-difference panel | Implemented |
| VTK DEM difference for mesh-initialized SCM | Implemented |
| Calibrated multi-ring RGB-D orbit capture | Implemented for final Go1 state |
| Closed-loop balance controller | Not implemented |
| Articulated Chrono leg dynamics | Not implemented |
| ROS 2 command bridge | Not implemented |
| Genesis MPM bridge | Implemented in companion `tera_splat`; parameter calibration pending |
| Gaussian transfer | Proposed only |
| Rigid offset hazard and reduced-order fall | Implemented |
| Rigid difficult terrain with completion and trunk tilt | Implemented approximation |
| Deformable SCM hills/valleys with completion and trunk tilt | Implemented approximation |
| Articulated contact-induced robot fall | Not implemented |

Latest verified PyVista traversal:

- historical output path: `quick_support_demo/outputs/videos/go1_velocity_trot_pyvista_dem_12p5kg.mp4`
- mass: `12.5 kg`
- SCM grid: `35 mm` smoke setting
- output: H.264, `1280x720`, 6 FPS, 66 frames, 11 seconds
- final active-node deformation: mean `13.62 mm`, maximum `27.61 mm`, 115 nodes

Latest verified rigid hazard artifact:

- historical output path: `quick_support_demo/outputs/videos/go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4`
- nominal robot mass: `12.5 kg`
- rigid block: center x offset `0.13 m`, height `0.13 m`
- event: front-right foot strike at `3.852 s`
- final lateral skid: `0.332 m`
- final trunk tilt: `90.0 deg`
- terrain deformation: none

Latest verified difficult-terrain artifact:

- historical output path: `quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4`
- historical side-view path: `quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_side_pyvista_12p5kg.mp4`
- nominal robot mass: `12.5 kg`
- course: three staggered rigid pads, `55-85 mm` high
- maximum commanded trunk tilt: `11.8 deg`
- completion: far floor at world `y=0.95 m`
- terrain deformation: none
- front-right post-pad stance clearance: `0.0 mm`

Pre-boundary-fix straight rolling artifact, retained for history:

- historical output path: `quick_support_demo/outputs/videos/go1_rolling_hills_valleys_scm_deformation_pyvista_dem_12p5kg.mp4`
- nominal robot mass: `12.5 kg`
- initial terrain elevation: `-64.2 mm` to `+76.9 mm`
- status: superseded because out-of-pit SCM queries lowered the spawn pose into
  the rigid perimeter; do not use its deformation or tilt metrics
- completion: far floor at world `y=0.95 m`
- output: H.264, `1280x720`, 6 FPS, 66 frames, 11 seconds
- right panel: DEM difference, `current terrain - initial terrain`

Latest verified forward-turn-forward artifact:

- historical output path: `quick_support_demo/outputs/videos/go1_rolling_scm_forward_turn_forward_pyvista_dem_12p5kg.mp4`
- sequence: `0.85 m` forward, `-90 deg` right turn at `0.8 rad/s`, `0.90 m` forward
- speed: `0.25 m/s`
- final pose: `(x=0.900, y=-0.249) m`, yaw `0.0 deg`
- deformation: mean `29.02 mm`, maximum `73.88 mm`, 137 nodes
- maximum commanded trunk tilt: `9.4 deg`
- output: H.264, `1280x720`, 6 FPS, 66 frames, 11 seconds

## Documentation map

Read only the documents relevant to the task, or follow them in order for a
complete system description.

1. [Overview and status](overview-and-status.md)
   Scope, capability matrix, repository layout, and what the demo proves.
2. [Architecture](architecture.md)
   Module boundaries, execution flows, state ownership, and data movement.
3. [Configuration reference](configuration-reference.md)
   Every YAML file, current values, units, smoke overrides, and key CLI flags.
4. [Physics and terrain](physics-and-terrain.md)
   Chrono system construction, SCM parameters, contact models, settling, and
   deformation measurement.
5. [Robots and locomotion](robots-and-locomotion.md)
   Go1 assets, support proxies, inverse kinematics, velocity gait, independent
   foot contacts, exact limits of the motion model, and the articulated-Go1
   replacement specification.
6. [Rendering and video](rendering-and-video.md)
   Matplotlib, PyVista/VTK, DEM visualization, graphics requirements, and video
   encoding.
7. [Planning and outputs](planning-and-outputs.md)
   Candidate costs, trial lifecycle, JSON and NumPy schemas, and artifact
   directory conventions.
8. [Installation and operations](installation-and-operations.md)
   Environment setup, routine commands, diagnostics, and expected runtimes.
9. [Validation and results](validation-and-results.md)
   Automated tests, verified videos, measured values, and validation methods.
10. [Limitations and supported claims](limitations-and-claims.md)
    Known model gaps, invalid interpretations, historical comparability, and
    what can be stated honestly.
11. [Roadmap and extensions](roadmap-and-extensions.md)
    Hazard/fall demonstration, articulated dynamics, ROS 2, Genesis MPM, and
    Gaussian scene work.
12. [Splat RGB-D capture](splat-rgbd-capture.md)
    Multi-level orbit sampling, RGB/depth formats, intrinsics, poses, and
    coordinate conventions.
13. [Getting started](getting-started.md)
    Concise environment setup, routine commands, and rendering entry points.

## Quick start

Activate the existing environment:

```bash
conda activate chrono_splat
```

Run the fast stationary support suite:

```bash
python run_demo.py --robot all --candidate all --smoke
```

Generate the nominal PyVista traversal and DEM video:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.055 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 30 --smoke \
  --output quick_support_demo/outputs/videos/go1_velocity_trot_pyvista_dem_12p5kg.mp4
```

Generate the rigid offset-hazard fall video:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --hazard \
  --hazard-offset-x 0.13 --hazard-height 0.13 \
  --hazard-slip-speed 0.55 --hazard-tip-rate 0.65 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.055 \
  --duration 7 --fps 8 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4
```

Generate the rigid difficult-terrain completion video:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --difficult-terrain \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4
```

Generate the rolling hills-and-valleys traversal with a synchronized DEM:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain --dem-panel \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-max-mm 90 --smoke \
  --output quick_support_demo/outputs/videos/go1_rolling_hills_valleys_scm_deformation_pyvista_dem_12p5kg.mp4
```

Here `--dem-panel` shows `current - initial` deformation of the rolling SCM
surface. `--dem-max-mm 90` gives useful contrast for the floor-clamped model.

Generate the forward-turn-forward maneuver on the same terrain:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain \
  --forward-turn-forward --vx 0.25 \
  --first-forward-distance 0.85 --turn-angle-deg -90 --turn-rate 0.8 \
  --second-forward-distance 0.90 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 140 --smoke \
  --output quick_support_demo/outputs/videos/go1_rolling_scm_forward_turn_forward_pyvista_dem_12p5kg.mp4
```

Add `--side-view` and use a separate output path for the lateral diagnostic:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --difficult-terrain --side-view \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_side_pyvista_12p5kg.mp4
```

Run all current automated tests:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
python -m unittest discover -s quick_support_demo/tests -v
```

## Documentation authority

When documents disagree, use this precedence:

1. Current source code and YAML configuration.
2. This `docs/` set.
3. The active documents in this directory.
4. Dated files in [`archive/`](archive/) for provenance only.

The date-sensitive verified state in this set is August 18, 2026.

## Legacy and proposal documents

The following files remain useful but are not the canonical current reference:

- [`quick-visual-demo-plan.md`](archive/quick-visual-demo-plan.md): original target,
  storyboard, and acceptance plan. Some milestones remain unfinished.
- [`current-system-state-2026-08-13.md`](archive/current-system-state-2026-08-13.md): earlier monolithic
  state record, including historical trials. It is useful for provenance but
  contains sections that predate velocity gait, tests, and VTK rendering.
- [`chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md`](archive/chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md):
  externally generated extension proposal. Genesis MPM and Gaussian transfer
  described there are not fully implemented in this repository.
- [`sim-only-validity-plan-2026-08-17.md`](archive/sim-only-validity-plan-2026-08-17.md):
  the initial cross-model validation plan. Use the companion project's current
  state for the latest completed bridge evidence.
