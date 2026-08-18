# Chronos Quick Support Demo: Comprehensive System State

> **Document status:** Historical monolithic snapshot retained for provenance.
> It contains sections that predate the current tests and PyVista renderer. For
> current documentation, start at [`docs/README.md`](../README.md).

Snapshot date: 2026-08-13

## 1. Executive summary

This repository currently contains a working PyChrono prototype for evaluating
locked-stance quadruped support on a deformable terrain patch. Its strongest
implemented capability is a deterministic Chrono SCM rollout in which a rigid
four-foot support body settles under gravity, changes a persistent terrain
heightfield, and produces sinkage and tilt measurements used by a small
support-aware planner.

The current system is not a dynamically walking robot simulator. The Go1 shown
in videos is an actual URDF-derived robot mesh. A velocity-command visual layer
now converts body-frame `(vx, vy, wz)` into open-loop trot foot paths and 12
joint targets. Stationary loading still uses one rigid four-pad body. Traversal
uses a kinematic collision-free trunk plus four independent stance/swing foot
bodies. There are no articulated Chrono leg links, joints, motors, balance
controller, or terrain-aware locomotion controller. Spot is still represented
by a box and four primitive foot pads.

There are two video paths:

- Chrono physics can now be rendered through either deterministic Matplotlib or
  a higher-quality PyVista/VTK backend with PBR materials, lights, shadows, and
  a native VTK DEM viewport.
- An Irrlicht path creates an interactive Chrono scene, but headless screenshots
  are black in the current Xvfb/Mesa setup and should not be treated as valid
  recorded output.

The current terrain can show SCM sinkage and permanent deformation. It does not
simulate individual grains. A deliberately heavy 50 kg Go1 variant produces
approximately 32.6 mm mean deformation over contacted terrain nodes and is the
clearest current visual artifact.

## 2. Original objective and current scope

The original plan in `quick-visual-demo-plan.md` asks for a short demonstration
of robot-conditioned planning over a central sand pit:

```text
same scene + same terrain + same objective
                  |
          robot-specific support
             /               \
     Go1 selects sand     Spot selects rigid
```

The longer-term visual goal stated during implementation is a video in which a
robot approaches or walks onto the pit and visible terrain deformation occurs.
No controller is required yet.

Current scope:

- PyChrono, not the C++ API.
- Rigid perimeter floor and central SCM patch.
- Four-foot locked support model released under gravity.
- Real Go1 visual geometry with standing and velocity-command articulated rendering.
- Two candidate poses and one shared planning objective.
- Offline JSON/NumPy outcomes and MP4 rendering.
- No dynamic walking, articulated rigid-body robot, ROS, sensors, Gaussian map, Genesis
  MPM, or granular DEM.

## 3. Capability matrix

| Capability | State | Notes |
|---|---|---|
| PyChrono environment | Working | `chrono_splat`, PyChrono 8.0.0 from ProjectChrono channel |
| Rigid perimeter floor | Working | Four fixed SMC-contact boxes around a central opening |
| SCM deformable pit | Working | 1.2 m x 1.2 m heightfield |
| Go1 support physics | Working proxy | One rigid body with four box contact pads |
| Go1 visual model | Working | URDF-derived standing cache plus runtime articulated transforms, simplified STL shells |
| Spot support physics | Working proxy | One rigid body with four pads |
| Spot visual model | Missing | No Spot URDF or meshes are present |
| Candidate planning | Working | Sand and rigid candidates with one shared objective |
| Go1/Spot split decision | Previously verified | Historical smoke run selected Go1/sand and Spot/rigid |
| Heightmap output | Working | Initial, loaded, and residual `.npy` files |
| 2D comparison video | Working | Matplotlib heightmap animation |
| 3D recorded video | Working | Chrono state rendered by Matplotlib |
| Synchronized DEM difference video | Working | Loading scene and signed current-minus-initial DEM from one Chrono timestep |
| Interactive Irrlicht | Starts | Requires a desktop display or X server |
| Headless Irrlicht capture | Broken | Captured BMP frames are black under Xvfb/Mesa |
| URDF parser in PyChrono | Unavailable | Installed build does not expose `pychrono.parsers` |
| Chrono Sensor | Unavailable | Python package stub exists, `_sensor` binary does not |
| Articulated legs/joints | Visual only | URDF link transforms animate from 12 targets; Chrono has no physical leg links or joints |
| Closed-loop walking controller | Missing | No balance, state estimation, force control, or terrain feedback |
| Velocity-command gait generation | Working visual layer | Body-frame `(vx, vy, wz)` produces open-loop trot foot paths and 12 visual joint targets |
| Independent gait foot contacts | Working approximation | Four Chrono foot bodies switch stance collision, share 12.5 kg, and settle dynamically in `z` |
| Articulated multibody gait physics | Missing | Feet are not connected to trunk by physical links, joints, or motors |
| Bulldozed soil berms | Available in API, disabled | SCM supports it but the builder does not enable it |
| Individual sand grains | Unsupported here | Requires a DEM/granular backend, not SCM |

## 4. Repository structure

```text
Chronos/
|-- docs/archive/quick-visual-demo-plan.md
|-- docs/archive/current-system-state-2026-08-13.md
|-- docs/getting-started.md            Short setup and command reference
|-- run_demo.py                        Trial/planning command-line entry point
`-- quick_support_demo/
    |-- assets/
    |   `-- go1/
    |       |-- urdf/go1.urdf
    |       |-- meshes/*.dae           Original Unitree visual assets
    |       |-- simplified_meshes/*.stl
    |       |-- render_cache/*.npz|*.obj
    |       |-- LICENSE
    |       `-- SOURCE.md
    |-- configs/
    |   |-- demo.yaml
    |   |-- world.yaml
    |   |-- terrain.yaml
    |   |-- go1.yaml
    |   |-- spot.yaml
    |   `-- candidates.yaml
    |-- chrono_demo/
    |   |-- chrono_import.py
    |   |-- build_world.py
    |   |-- build_scm_pit.py
    |   |-- build_support_proxy.py
    |   |-- run_support_trial.py
    |   |-- outcomes.py
    |   `-- run_visualizer.py
    |-- planning/
    |   |-- support_cost.py
    |   `-- select_pose.py
    |-- robot_assets/
    |   `-- go1.py
    |-- overlays/
    |   |-- make_preview_video.py
    |   `-- make_chrono_3d_video.py
    `-- outputs/
        |-- trials/<timestamp>/...
        |-- frames/<timestamp>/...
        `-- videos/*.mp4
```

The directories anticipated by the original plan for Spot, target assets, and
textures either do not exist or have no populated assets. The target currently
uses a yellow primitive box.

## 5. Runtime environment

Use the existing Conda environment:

```bash
conda activate chrono_splat
```

Important installed package versions:

| Package | Version | Purpose |
|---|---:|---|
| `pychrono` | 8.0.0 | Chrono core, vehicle/SCM, Irrlicht, and robot bindings |
| `numpy` | 1.26.4 | Heightmaps, transforms, and metrics |
| `pyyaml` | 6.0.3 | Configuration loading |
| `matplotlib` | 3.10.9 | Reliable headless 3D and 2D rendering |
| `imageio` | 2.37.0 | Video writing |
| `imageio-ffmpeg` | 0.6.0 | H.264 encoder access |
| `opencv` | 4.12.0 | Installed for overlays/video work |
| `trimesh` | 5.0.0 | Robot mesh loading and assembly |
| `pycollada` | 0.9.3 | Collada support |
| `fast-simplification` | 0.1.13 | Mesh decimation |
| `xorg-xvfb-server` | 21.1.24 | Virtual X server for Irrlicht experiments |
| `xvfbwrapper` | 0.2.29 | Python Xvfb wrapper |
| `mesalib` | 26.1.6 | Software OpenGL attempt |

PyChrono must come from the ProjectChrono channel in this environment. The
tested conda-forge PyChrono 10 package did not expose
`pychrono.vehicle.SCMTerrain`.

Reinstallation commands:

```bash
conda install -n chrono_splat -c projectchrono -c conda-forge pychrono=8.0.0
conda install -n chrono_splat -c conda-forge \
  numpy pyyaml opencv matplotlib imageio imageio-ffmpeg \
  trimesh pycollada fast-simplification
```

## 6. Configuration snapshot

### World

`quick_support_demo/configs/world.yaml` defines:

| Setting | Value |
|---|---:|
| Floor size | 3.0 m x 3.0 m |
| Gravity | `[0, 0, -9.81]` m/s2 |
| Configured timestep | 0.0005 s |
| Configured settle limit | 3.0 s |
| Linear settle threshold | 0.005 m/s |
| Angular settle threshold | 0.01 rad/s |
| Floor thickness | 0.08 m |
| Floor friction | 0.8 |

The system is `ChSystemSMC` using the Bullet collision backend. Contact
materials use a deliberately soft Young modulus of `1.0e6`, restitution `0.05`,
and friction supplied by the caller. The softer material was selected for
stable rigid-floor contact in this prototype.

### Terrain

`quick_support_demo/configs/terrain.yaml` defines:

| Setting | Value |
|---|---:|
| Patch size | 1.2 m x 1.2 m |
| Configured grid spacing | 0.01 m |
| Surface elevation | 0.0 m |
| Bekker `Kphi` | 800,000 |
| Bekker `Kc` | 25,000 |
| Bekker exponent `n` | 1.1 |
| Mohr cohesion | 1,000 Pa |
| Mohr friction setting | 28 degrees in YAML |
| Janosi shear distance | 0.01 m |
| Elastic stiffness | 20,000,000 Pa/m |
| Damping | 30,000 Pa s/m |

These values were chosen for visible deformation, not calibrated against a
specific sand. SCM is a pressure-sinkage and shear model, not a particle model.

### Robots

Go1 configuration:

| Setting | Value |
|---|---:|
| Nominal mass | 12.5 kg |
| Payload | 0 kg |
| Stance length | 0.3762 m |
| Stance width | 0.2535 m |
| Proxy foot radius | 0.035 m |
| Proxy foot height | 0.035 m |
| Proxy COM height | 0.30 m |
| Initial clearance | 0.005 m |

Spot proxy configuration:

| Setting | Value |
|---|---:|
| Nominal mass | 32.5 kg |
| Payload | 0 kg |
| Stance length | 0.70 m |
| Stance width | 0.36 m |
| Proxy foot radius | 0.032 m |
| Proxy foot height | 0.04 m |
| Proxy COM height | 0.38 m |
| Initial clearance | 0.005 m |

### Candidate poses and planner

| Candidate | Base XY | View cost | Path cost |
|---|---:|---:|---:|
| Sand | `[0.0, -0.05]` | 0.0 | 0.0 |
| Rigid | `[-1.05, -0.20]` | 0.5 | 0.5 |

The objective is:

```text
J = 1.0 * view_cost
  + 1.0 * path_cost
  + 45.0 * max_foot_sinkage_m
  + 3.0 * max_abs_body_tilt_rad
  + 0.0 * uncertainty
```

The lower-cost candidate is selected. View and path costs are manually defined;
support terms come from Chrono trial outcomes.

## 7. Physics model

### World construction

The floor is four fixed boxes surrounding the SCM patch. There is no rigid
plane directly under the terrain surface, avoiding simultaneous rigid and SCM
contact at `z = 0`.

### SCM construction

`build_scm_pit.py`:

1. Creates `pychrono.vehicle.SCMTerrain`.
2. Places its reference plane at the configured elevation.
3. Initializes a flat square grid.
4. Applies eight soil parameters.
5. Configures sinkage pseudo-color plotting for Irrlicht.

Bulldozing is currently not enabled. Therefore, the terrain depresses and
retains ruts but does not intentionally form redistributed side berms through
SCM's erosion/flow feature.

### Support proxy

Each robot is physically represented by one `ChBody`:

- Total mass is robot mass plus payload.
- Inertia is that of the configured body box.
- Four collision boxes are rigidly attached at stance offsets.
- Each box has the same area as the configured circular foot:
  `side = sqrt(pi * radius^2)`.
- The whole assembly can translate and rotate under gravity.
- Individual legs cannot move or redistribute load through joints.

This model can show overall settlement, differential foot settlement, roll,
and pitch. It cannot show leg compliance, joint limits, active stabilization,
slip recovery, or gait dynamics.

### Trial lifecycle

For each candidate:

1. Build a new Chrono system and fresh SCM terrain.
2. Sample the initial heightmap.
3. Place the proxy at the candidate position.
4. Simulate until the configured time limit or sustained velocity thresholds.
5. Sample the loaded heightmap.
6. Measure foot-bottom depth, body roll/pitch, and COM height change.
7. Remove the body.
8. Advance terrain for another 0.5 s.
9. Sample the residual heightmap.
10. Save JSON metrics and NumPy arrays.

Each candidate uses a fresh world. Candidate results are counterfactual
rollouts, not sequential interactions with one persistent pit.

## 8. Robot visual asset pipeline

### Sources and license

The Go1 URDF and original Collada files come from Unitree's public
`unitreerobotics/unitree_ros` repository. Clean STL link meshes come from
Google DeepMind's MuJoCo Menagerie Go1 model, which is derived from the same
public URDF. The included license is BSD-3-Clause.

See:

- `quick_support_demo/assets/go1/SOURCE.md`
- `quick_support_demo/assets/go1/LICENSE`

### Standing-pose assembly

`robot_assets/go1.py` parses the URDF XML itself because the installed PyChrono
build has no URDF parser module. It computes a kinematic tree using a fixed home
pose:

```text
hip abduction:  0.0 rad
hip/thigh:      0.9 rad
knee/calf:     -1.8 rad
```

The asset builder then:

1. Loads each simplified STL link.
2. Merges vertices and takes a convex visual shell.
3. Decimates the shell.
4. Applies URDF link transforms and model-specific hip corrections.
5. Aligns average visual foot height with the hidden proxy feet.
6. Adds four small visual foot spheres.
7. Exports a combined NPZ mesh for Matplotlib.
8. Exports a combined OBJ mesh for Chrono/Irrlicht.

The standing cache contains approximately 4,077 vertices and 8,086 faces. The
stationary renderer uses this baked mesh. The traversal renderer instead
recomputes link transforms from 12 joint targets at each frame. Both modes are
still attached to one rigid Chrono physics body, so neither is an articulated
simulation model.

## 9. Rendering paths

### Reliable Chrono-driven Matplotlib video

`make_chrono_3d_video.py` advances a live Chrono system and samples SCM at every
requested video frame. Matplotlib draws:

- a heightfield using the actual sampled node elevations;
- deterministic dirt-like albedo variation;
- slope-based lighting;
- the rigid perimeter floor and inspection target;
- the Go1 assembled mesh or Spot proxy;
- live mean/max interior-node deformation metrics.

The approach is scripted. Before contact, the rigid robot body is fixed,
collision-disabled, and smoothly translated from the start to the sand pose.
At the release time, it is made dynamic, collision is enabled, and gravity and
SCM determine the settlement.

The heavy video uses `--mass-scale 4`, scaling Go1 from 12.5 kg to 50 kg. This
changes Chrono mass and inertia, so the greater sinkage is physically computed
by SCM rather than fabricated in the main sequence.

During only the final footprint-reveal shot:

- the visual robot fades to 28% opacity;
- the camera becomes steeper and tighter;
- interior terrain displacement smoothly increases to 3x vertical display
  exaggeration;
- the on-screen label explicitly reports the exaggeration;
- the displayed millimeter metrics remain unscaled Chrono values.

With `--dem-panel`, the renderer instead keeps the loading scene at true
vertical scale and places a top-down DEM difference beside it. The difference
is `(current surface - initial surface)` in millimeters: red values are
subsidence and blue values are uplift. Foot-center markers provide spatial
registration. The outer SCM node ring is masked to exclude the known
patch-boundary/base-level artifact. `--dem-max-mm` sets a fixed subsidence color
limit for all frames; the uplift limit is one third of that magnitude.

### PyVista / VTK video

`--renderer pyvista` runs the same Chrono stepping, gait, independent-foot
contacts, SCM sampling, and deformation metrics through a persistent VTK
scene. The backend updates the terrain and articulated Go1 meshes in place and
adds granular soil albedo, PBR materials, two-light illumination, shadows,
FXAA, and a native top-down DEM viewport. Its DEM uses a symmetric fixed color
range so zero deformation remains white. Matplotlib remains the default.

The installed VTK wheel renders offscreen through EGL on this workstation when
the process has graphics-device access. Systems without EGL or OSMesa need a
working X display/Xvfb or a headless-enabled VTK build.

### 2D preview

`make_preview_video.py` reads saved trial arrays and animates interpolation from
initial to loaded heightmaps. It shows Go1 and Spot side by side with selection,
sinkage, and tilt labels. It is not a live 3D Chrono rendering.

### Irrlicht

`run_visualizer.py` creates a true Chrono Irrlicht scene with SCM's visualization
mesh, candidate markers, target, and robot body. Interactive startup works with
an appropriate X display. Under Xvfb plus Mesa software rendering,
`WriteImageToFile` has repeatedly produced black frames. Existing files named
`*_irrlicht.mp4` and BMPs under `outputs/frames` are failed diagnostics, not
valid demonstration videos.

## 10. Outputs and measured behavior

### Current video artifacts

| File | Contents | Verified metadata |
|---|---|---|
| `go1_chrono_dirt_visual.mp4` | Nominal Go1, dirt shading, Chrono SCM | 6 s, 1280x720, 12 FPS, 72 frames |
| `go1_chrono_dirt_heavy_4x.mp4` | 50 kg Go1, close view, footprint reveal | 6 s, 1280x720, 6 FPS, 36 frames |
| `go1_loading_dem_difference_4x.mp4` | 50 kg Go1 loading beside synchronized signed DEM change | 6 s, 1280x720, 6 FPS, 36 frames |
| `go1_loading_dem_nominal_12p5kg.mp4` | Corrected friction, 12.5 kg Go1, loading beside signed DEM change, explicit 35 mm smoke grid | 6 s, 1280x720, 6 FPS, 36 frames |
| `go1_velocity_trot_traverse_12p5kg.mp4` | 12.5 kg proxy traverses pit with velocity-command articulated visual trot and DEM | 11 s, 1280x720, 6 FPS, 66 frames |
| `go1_velocity_trot_traverse_independent_feet_12p5kg.mp4` | Open-loop trot with independent stance/swing Chrono foot contacts and discrete footprints | 11 s, 1280x720, 6 FPS, 66 frames |
| `go1_velocity_trot_pyvista_dem_12p5kg.mp4` | Same nominal-mass traversal rendered with PBR PyVista scene and native VTK DEM | 11 s, 1280x720, 6 FPS, 66 frames |
| `20260812_165429_preview.mp4` | Go1/Spot 2D split comparison | 6 s, 1280x720, 24 FPS, 144 frames |

The first three Chrono videos in this table predate the friction-unit
correction and are retained only as visual-development artifacts. The
`go1_loading_dem_nominal_12p5kg.mp4` artifact is the corrected nominal-mass
comparison.

### Corrected nominal-mass Go1 smoke trial

Output: `quick_support_demo/outputs/trials/20260812_204433/`

| Metric | Value |
|---|---:|
| Selected candidate | Sand |
| Configured physical proxy mass | 12.5 kg |
| Smoke grid spacing | 40 mm |
| Four foot sinkages | 18.91, 4.56, 18.91, 4.56 mm |
| Mean foot sinkage | 11.73 mm |
| Maximum foot sinkage | 18.91 mm |
| Maximum body tilt | 0.05666 rad / 3.25 degrees |
| COM height change | -17.21 mm |

From that trial's loaded heightmap, excluding the outermost SCM grid ring:

| Terrain statistic | Value |
|---|---:|
| Mean over displaced interior nodes | 11.16 mm |
| Maximum interior deformation | 21.45 mm |
| Mean over the entire interior pit | 0.159 mm |
| Displaced interior nodes | 12 of 841 |

### Velocity-command traversal

`go1_velocity_trot_traverse_independent_feet_12p5kg.mp4` commands body-frame
`(vx, vy, wz) = (0.25, 0.0, 0.0)` with a 1.6 Hz diagonal trot and 55 mm visual
swing clearance. The proxy crosses from `y = -1.10 m` to `y = +1.10 m`.

| Terrain statistic | Value |
|---|---:|
| Physical proxy mass | 12.5 kg |
| Mean over displaced interior nodes | 13.62 mm |
| Maximum interior deformation | 27.61 mm |
| Displaced interior nodes | 115 |

Each stance foot is an independent Chrono collision body. The total 12.5 kg
load is divided equally over the active stance set, so a diagonal two-foot
phase applies 6.25 kg per contact. Swing feet are fixed, raised, and
collision-disabled. Stance feet preserve dynamic vertical motion while their
planar locations follow the open-loop gait. This produces discrete footprints,
but it is not force-balanced articulated robot dynamics.

The outermost queried SCM ring reports an approximately 80 mm level difference
that is a patch-boundary/base-level artifact, not robot deformation. Video
deformation metrics intentionally exclude that ring.

### Heavy visual trial

The 4x-mass video uses a 50 kg Go1 proxy and reports:

| Terrain statistic | Value |
|---|---:|
| Mean over displaced interior nodes | 32.64 mm |
| Maximum interior deformation | 33.50 mm |
| Displaced interior nodes | 16 |

### Historical full planning comparison

The last recorded all-robot split comparison is under
`quick_support_demo/outputs/trials/20260812_165429/`:

| Robot | Sand max sinkage | Sand tilt | Sand cost | Rigid cost | Selected |
|---|---:|---:|---:|---:|---|
| Go1 | 16.99 mm | 0.00716 rad | 0.786 | 1.046 | Sand |
| Spot | 115.07 mm | 0.02565 rad | 5.255 | 1.088 | Rigid |

This run demonstrates the intended one-objective/two-decisions behavior, but it
predates later Go1 stance and visual-pipeline changes. It has not been rerun
after every current code/configuration edit and should be described as a
historical verified result, not a fresh calibration of the exact current tree.

## 11. Output schema

Trial directory layout:

```text
outputs/trials/<timestamp>/
`-- <robot>/
    |-- summary.json
    |-- sand/
    |   |-- outcome.json
    |   |-- initial_heightmap_m.npy
    |   |-- loaded_heightmap_m.npy
    |   `-- residual_heightmap_m.npy
    `-- rigid/
        `-- ...
```

`outcome.json` includes:

- robot and candidate names;
- four foot sinkages;
- body roll and pitch;
- COM height change;
- simulation runtime;
- selected candidate and total objective cost;
- heightmap shapes;
- maximum/mean foot sinkage;
- maximum absolute tilt.

The actual heightmaps are stored separately as float NumPy arrays in meters.

## 12. Commands

### Fast all-robot planning smoke test

```bash
conda run -n chrono_splat python run_demo.py \
  --robot all --candidate all --smoke
```

Smoke mode changes:

- timestep from 0.0005 s to 0.001 s;
- settle limit from 3.0 s to 0.6 s;
- grid spacing from 0.01 m to 0.04 m.

### Single Go1 sand trial

```bash
conda run -n chrono_splat python run_demo.py \
  --robot go1 --candidate sand --smoke
```

### Configured higher-resolution trial

```bash
conda run -n chrono_splat python run_demo.py \
  --robot all --candidate all
```

### Nominal 3D video

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --duration 6.0 --fps 12
```

### Heavy 3D video

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --mass-scale 4 --duration 6.0 --fps 6 \
  --output quick_support_demo/outputs/videos/go1_chrono_dirt_heavy_4x.mp4
```

### Loading plus DEM difference video

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --duration 6.0 --fps 6 \
  --dem-panel --dem-max-mm 12 --smoke \
  --output quick_support_demo/outputs/videos/go1_loading_dem_nominal_12p5kg.mp4
```

### Velocity-command traversal

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --traverse --vx 0.25 --vy 0 --wz 0 \
  --gait-frequency 1.6 --step-height 0.055 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 30 --smoke \
  --output quick_support_demo/outputs/videos/go1_velocity_trot_traverse_independent_feet_12p5kg.mp4
```

### 2D planning preview

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos \
conda run -n chrono_splat python \
  -m quick_support_demo.overlays.make_preview_video
```

### Interactive Irrlicht attempt

```bash
conda run -n chrono_splat python \
  -m quick_support_demo.chrono_demo.run_visualizer \
  --robot go1 --smoke
```

## 13. Important known issues

### 13.1 SCM friction-angle correction

`build_scm_pit.py` now passes the YAML `mohr_friction_deg` value directly to
`SCMTerrain.SetSoilParameters`, whose installed PyChrono 8 header specifies
degrees. Outputs generated before this correction used approximately `0.489`
instead of `28.0`; those historical outputs remain visual artifacts and must
not be calibration targets.

### 13.2 Video resolution selection

The renderer now defaults to the configured `0.01 m` grid and exposes mutually
exclusive `--smoke` and `--full-res` options. `--smoke` uses a `0.035 m` grid
and `0.001 s` timestep. The corrected nominal-mass comparison uses this explicit
coarse mode and labels its grid in-frame. A full-resolution video attempt was
too slow for interactive generation and was not retained.

### 13.3 Rigid-candidate "sinkage" is not terrain deformation

Foot sinkage is computed as `max(0, -foot_bottom_z)`. On the rigid floor, small
negative values primarily reflect compliant SMC contact penetration and the
chosen material stiffness. They should be described as contact penetration,
not rigid terrain deformation.

### 13.4 Robot and contact model are not geometrically identical

The stationary Go1 visual feet are aligned to hidden support pads. Traversal
uses separate Chrono contact-foot bodies, but:

- visual links are convexified and decimated;
- contact pads are square boxes with circular-equivalent area;
- traversal feet are not constrained to the trunk by physical leg links;
- mass distribution uses one box inertia rather than URDF link inertias.

This is suitable for a support proxy demonstration, not a validated Go1 model.

### 13.5 Historical results span different configurations

Outputs under `outputs/trials` were produced throughout development. They use
different stance geometry, grid spacing, contact settings, and code revisions.
Timestamped output directories are not one homogeneous benchmark dataset.

### 13.6 No automated tests

There is no test suite. Verification has consisted of:

- Python compilation checks;
- smoke simulations;
- output JSON inspection;
- FFprobe metadata checks;
- manual frame inspection.

### 13.7 Rendering limitations

- Matplotlib is deterministic and reliable but slow.
- Matplotlib does not provide physically based dirt materials, shadows, or a
  high-quality real-time camera.
- Irrlicht's headless screenshot route is black in this environment.
- Existing failed Irrlicht MP4/BMP files may be mistaken for valid outputs.

## 14. Sand behavior controls

SCM parameters available now:

| Parameter | Physical role | Typical loose-sand direction |
|---|---|---|
| Bekker `Kphi` | Frictional pressure-sinkage stiffness | Lower for deeper sinkage |
| Bekker `Kc` | Cohesive pressure-sinkage stiffness | Near zero for dry sand |
| Bekker `n` | Pressure-sinkage nonlinearity | Often around 0.9 to 1.2 |
| Mohr cohesion | Pressure-independent shear strength | Near zero for dry sand |
| Mohr friction | Internal shear resistance | Roughly 28 to 35 degrees as a starting range |
| Janosi shear distance | Shear displacement to mobilize strength | Larger for more shear displacement/slip |
| Elastic stiffness | Reversible response before plastic yield | Keep above `Kphi`; higher favors persistent plastic tracks after yield |
| Damping | Vertical rate resistance | Higher reduces bouncing and rapid penetration |
| Grid spacing | Spatial resolution | Smaller produces better footprint shape at higher cost |

The installed API also exposes:

```python
terrain.EnableBulldozing(True)
terrain.SetBulldozingParameters(
    erosion_angle,
    flow_factor,
    erosion_iterations,
    erosion_propagations,
)
```

Bulldozing is the most relevant next SCM feature for a granular-looking result
because it moves displaced volume into raised material near ruts. The
friction-angle bug is fixed; bulldozing should remain disabled until a stable
full-resolution baseline is recorded.

SCM still cannot produce individually resolved grains, splashing particles, or
true grain avalanches. Those effects require a DEM/granular solver. The current
PyChrono environment does not expose Chrono GPU or multicore granular modules.

## 15. Acceptance status against the original plan

### Milestone 1: terrain contact

- Rigid perimeter floor: complete.
- Central SCM patch: complete.
- Stable settlement: complete with four-foot proxy.
- Reset through fresh-world construction: complete.
- Initial/loaded/residual heightmaps: complete.
- Dedicated one-platen validation artifact: not retained as a distinct module.

### Milestone 2: four-foot proxy

- Go1 mass/stance proxy: complete.
- Spot mass/stance proxy: complete.
- Sinkage and tilt output: complete.
- Heavier loading produces different deformation: verified with 4x Go1.

### Milestone 3: visual assets

- Go1 URDF and meshes: partially complete; visual-only assembly.
- Go1 nominal standing pose: complete visually.
- Go1 primitive foot contacts: complete.
- Spot URDF and meshes: missing.
- Articulated links and URDF mass distribution: missing.

### Milestone 4: planning display

- Two candidate poses: complete in configuration.
- Manual view/path costs: complete.
- Shared support-aware objective: complete.
- Different historical Go1/Spot selections: verified.
- Polished combined 3D comparison: incomplete.

### Milestone 5: Gaussian/MPM extension

Not started.

### Original acceptance criteria

| Criterion | State |
|---|---|
| Script can select Go1/Spot and reset world | Met through fresh trial construction |
| Both robots appear at correct metric scale | Partial; Go1 mesh, Spot proxy only |
| Stable four-foot rigid contact | Met at proxy level |
| Visible SCM settlement | Met, clearest in heavy/reveal video |
| Go1 and Spot outcomes differ | Historically met |
| Shared objective produces different choices | Historically met |
| Final view shows robot, pit, target, candidates, result | Partial across separate video paths |
| Deterministic replay from fixed configuration | Mostly met; no formal reproducibility test |

## 16. Recommended next work, in order

### Phase A: establish a trustworthy SCM baseline

1. Completed: pass SCM friction in degrees.
2. Completed: expose explicit `--smoke` and `--full-res` video paths, with full
   resolution as the default.
3. Completed: reject non-finite heightmaps and unexpected sampled shapes.
4. Enable bulldozing through YAML-configured parameters.
5. Start with low cohesion and `Kc` near zero for dry-sand behavior.
6. Run a small parameter sweep at nominal Go1 mass.
7. Save a manifest with every output containing exact config values and code
   revision information.
8. Add deterministic-repeat and monotonic mass/sinkage regression checks.

### Phase B: make deformation legible without relying on 4x mass

1. Render with the configured 10 mm grid.
2. Add a side or low-angle close-up focused on one foot.
3. Show a pre/post terrain difference inset or contour overlay.
4. Retain the labeled footprint-reveal shot.
5. Use SCM bulldozing to form visible side berms.

### Phase C: finish robot assets

1. Add the BSD-licensed Spot URDF and meshes.
2. Build the same visual-cache path for Spot.
3. Validate model extents, standing pose, foot alignment, and mass.
4. Regenerate the same-scene Go1/Spot planning comparison using current code.

### Phase D: articulated no-controller robot

1. Implement or obtain a URDF-to-Chrono loader because `pychrono.parsers` is
   unavailable in this build.
2. Create one Chrono body per URDF link.
3. Add revolute joints with URDF limits and inertias.
4. Initialize a standing configuration.
5. Lock joints or use stiff passive/position constraints only for settling.
6. Keep simplified primitive foot contacts.
7. Validate against the current proxy before attempting motion.

### Phase E: scripted visual stepping, still without a locomotion controller

A video that visually "walks onto" the pit without claiming controller
fidelity can use a prescribed gait trajectory:

1. Completed visually: convert `(vx, vy, wz)` into diagonal trot stance/swing
   trajectories and 12 inverse-kinematics joint targets.
2. Completed visually: move the rendered feet through independent swing paths.
3. Completed approximation: assign independent Chrono collision bodies to feet,
   disable swing collision, and distribute the full mass over stance contacts.
4. Pending multibody physics: connect trunk and feet through physical links,
   joints, and motors instead of prescribing foot planar motion.
5. Add balance and terrain feedback only as a separate later milestone.

## 17. Honest interpretation of the current demo

The current demo supports this claim:

> A robot-conditioned rigid support proxy, simulated with Chrono SCM, can
> produce measurable persistent terrain deformation and can feed those support
> outcomes into a shared candidate-selection objective.

It does not yet support these stronger claims:

- A Go1 or Spot locomotion controller can traverse the sand.
- The URDF robot's articulated dynamics were simulated.
- The SCM parameters represent a calibrated real sand.
- The terrain is grain-resolved.
- The rendered 3x reveal is true geometric scale.
- The historical Go1/Spot split has been revalidated after every current edit.

That distinction should remain explicit in videos, figures, and reports until
the corresponding implementation milestones are complete.
