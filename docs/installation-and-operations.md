# Installation and Operations

[Documentation index](README.md)

## Environment

The expected conda environment is `chrono_splat`, currently using Python 3.10.
Important installed versions:

| Package | Version | Source |
|---|---:|---|
| PyChrono | `8.0.0` | ProjectChrono channel |
| `chrono` package | `10.0.0` | conda-forge |
| NumPy | `1.26.4` | conda-forge |
| Matplotlib | `3.10.9` | conda-forge |
| ImageIO | `2.37.0` | conda-forge |
| imageio-ffmpeg | `0.6.0` | conda-forge |
| trimesh | `5.0.0` | conda-forge |
| PyVista | `0.48.4` | pip |
| VTK | `9.6.2` | pip |

The PyChrono 8 ProjectChrono build is required by the current code because it
exposes `pychrono.vehicle.SCMTerrain`. The separately installed conda-forge
`chrono` package should not be assumed to provide the same Python modules.

Newton and Warp are not runtime dependencies of this repository. The companion
`tera_splat` Newton branch uses the separate environment
`/data/christoa/conda/envs/newton_splat`, currently Python 3.11.15, Newton
1.5.1, and Warp 1.17.0. The Chrono oracle commands and `chrono_splat`
environment remain the reference producer.

## Installation commands

Representative environment setup:

```bash
conda install -n chrono_splat -c conda-forge \
  numpy pyyaml opencv matplotlib imageio imageio-ffmpeg \
  trimesh pycollada fast-simplification

conda install -n chrono_splat -c projectchrono -c conda-forge pychrono=8.0.0

conda run -n chrono_splat pip install pyvista vtk
```

## Writable caches

Use writable cache locations in automated/headless sessions:

```bash
export MPLCONFIGDIR=/tmp/matplotlib-chronos
export XDG_CACHE_HOME=/tmp/chrono-cache
```

## Stationary experiments

Fast all-robot, all-candidate validation:

```bash
conda run -n chrono_splat python run_demo.py \
  --robot all --candidate all --smoke
```

Single Go1 sand trial:

```bash
conda run -n chrono_splat python run_demo.py \
  --robot go1 --candidate sand --smoke
```

Configured 10 mm grid:

```bash
conda run -n chrono_splat python run_demo.py \
  --robot all --candidate all
```

## Matplotlib traversal

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer matplotlib --traverse \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.055 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 30 --smoke \
  --output quick_support_demo/outputs/videos/go1_velocity_trot_traverse_independent_feet_12p5kg.mp4
```

## PyVista traversal

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.055 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 30 --smoke \
  --output quick_support_demo/outputs/videos/go1_velocity_trot_pyvista_dem_12p5kg.mp4
```

## Static loading video

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --duration 6 --fps 6 --dem-panel \
  --dem-max-mm 12 --smoke
```

## Rigid offset-hazard fall

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --hazard \
  --hazard-offset-x 0.13 --hazard-height 0.13 \
  --hazard-slip-speed 0.55 --hazard-tip-rate 0.65 \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.055 \
  --duration 7 --fps 8 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4
```

This run uses nominal `12.5 kg` mass and no deformable terrain. The slip speed
and tip rate are reduced-order release inputs, not controller outputs. `--smoke`
is retained for command consistency but has no SCM grid to modify in hazard mode.

## Heavy visual-only variant

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --mass-scale 4 --duration 6 --fps 6 --smoke
```

`--mass-scale 4` changes physical mass to `50 kg`. It is not a nominal Go1
result and must be labeled as a visualization stress case.

## Rigid difficult-terrain completion

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --difficult-terrain \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4
```

The higher `0.10 m` swing clearance is intentional because the tallest pad is
`0.085 m`. This mode has no SCM and rejects `--dem-panel`.

Side-view diagnostic:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --difficult-terrain --side-view \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --smoke \
  --output quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_side_pyvista_12p5kg.mp4
```

## Deformable rolling terrain with DEM difference

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain --dem-panel \
  --difficult-max-tilt-deg 14 \
  --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-max-mm 90 --smoke \
  --output quick_support_demo/outputs/videos/go1_rolling_hills_valleys_scm_deformation_pyvista_dem_12p5kg.mp4
```

No `--side-view` is used: this is a perspective scene plus DEM. A `90 mm` range
provides clear contrast for the corrected floor-clamped support model.

## Forward-turn-forward maneuver

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain \
  --forward-turn-forward --vx 0.25 \
  --first-forward-distance 0.85 --turn-angle-deg -90 --turn-rate 0.8 \
  --second-forward-distance 0.90 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 140 --smoke \
  --output quick_support_demo/outputs/videos/go1_rolling_scm_forward_turn_forward_pyvista_dem_12p5kg.mp4
```

Use a positive `--turn-angle-deg` for a left turn. Ensure `--duration` exceeds
the `0.8 s` warmup plus both distance/speed durations and angle/rate duration
if the final pose must be reached.

## Tests

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python \
  -m unittest discover -s quick_support_demo/tests -v
```

## Diagnostics

Check imports and versions:

```bash
conda run -n chrono_splat python -c \
  'import pychrono, pychrono.vehicle, pyvista, vtk; print(pyvista.__version__, vtk.vtkVersion.GetVTKVersion())'
```

Check a video:

```bash
ffprobe -v error \
  -show_entries format=duration,size \
  -show_entries stream=codec_name,pix_fmt,width,height,avg_frame_rate,nb_frames \
  -of default=noprint_wrappers=1 <video.mp4>
```

If VTK emits EGL or OSMesa errors:

1. verify graphics-device access;
2. verify `DISPLAY` for X-backed builds;
3. use an EGL/OSMesa-enabled VTK build or Xvfb;
4. use `--renderer matplotlib` as the portable fallback.

## Runtime expectations

Runtime depends strongly on SCM grid, physics duration, frame count, renderer,
resolution, PBR, and shadows. On this workstation the verified 11-second,
66-frame, 1280x720 PyVista traversal took several minutes. Full `10 mm` SCM
resolution is substantially slower than the `35 mm` smoke grid.
