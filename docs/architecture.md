# Architecture

[Documentation index](README.md)

## Design principle

Simulation, motion generation, planning, persistence, and rendering are kept as
separate concerns. Chrono owns physical state. The gait module generates target
kinematics. Renderers consume sampled state but do not change physics.

## Component map

```text
YAML configuration
       |
       v
config.load_demo_config
       |
       +------------------------+
       |                        |
       v                        v
stationary trial          traversal video
run_support_trial         make_chrono_3d_video
       |                        |
       v                        +--> TrotGait / VelocityCommand
Chrono system + SCM             +--> optional ForwardTurnForward scheduler
       |                        +--> IndependentFeet
       |                        +--> normal: Chrono system + SCM
       |                        +--> hazard: rigid plate + RigidHazard
       |                        +--> difficult: rigid plate + DifficultCourse
       |                        +--> rolling: mesh-initialized SCM + RollingCourse
       v                        |
SupportOutcome                  +--> sampled height map
       |                        +--> robot/gait state
       v                        |
objective_cost                  +--> Matplotlib renderer
select_candidate                `--> PyVistaFrameRenderer
       |
       v
JSON + NPY outputs
```

## Module responsibilities

### Configuration

- `quick_support_demo/config.py`
  Resolves project-relative paths and loads the composed YAML configuration.
- `quick_support_demo/configs/*.yaml`
  Owns world, terrain, candidate, planning, and robot parameters.

### Chrono world and terrain

- `chrono_demo/chrono_import.py`
  Imports the PyChrono modules required by the repository.
- `chrono_demo/build_world.py`
  Creates `ChSystemSMC`, contact materials, rigid floor boxes, and gravity.
- `chrono_demo/build_scm_pit.py`
  Creates `SCMTerrain`, applies soil parameters, and samples the height map.
- `chrono_demo/hazard.py`
  Defines the fixed block dimensions, adds its collision body, tests geometric
  foot intersection, and creates the constant zero height map used by renderers
  when SCM is absent.
- `chrono_demo/difficult_terrain.py`
  Defines rigid pad and rolling-heightfield geometry, creates the Chrono
  collision bodies, samples both course types, and fits bounded support-plane
  roll/pitch from commanded foot poses.

### Contact models

- `chrono_demo/build_support_proxy.py`
  Creates the stationary rigid support body and optional four foot collision
  boxes. The Go1 visual mesh can be attached to that one body. Hazard mode also
  enables a trunk collision box so the released body contacts the rigid course.
- `chrono_demo/independent_feet.py`
  Creates and updates four independent foot bodies for traversal. It changes
  collision and fixed state as feet enter stance or swing.

### Experiments and outcomes

- `chrono_demo/run_support_trial.py`
  Runs one stationary robot/candidate simulation and returns `SupportOutcome`.
- `chrono_demo/outcomes.py`
  Defines the result dataclass, derived metrics, JSON conversion, and height-map
  persistence.
- `run_demo.py`
  Runs requested robot/candidate combinations, selects a candidate, and writes
  timestamped output directories.

### Motion

- `motion/forward_turn_forward.py`
  Converts configured distances, signed turn angle, speed, and turn rate into
  `forward 1`, `turn left/right`, `forward 2`, and `complete` velocity-command
  phases. It does not integrate world pose or inspect terrain.
- `motion/velocity_gait.py`
  Defines `VelocityCommand`, `GaitState`, diagonal trot phase scheduling,
  analytical leg IK, and forward kinematics used by tests.

The video loop integrates the active command into world x/y/yaw. For the
default right-turn maneuver, heading changes from world `+y` to `+x`; the
normal straight-traversal y endpoint clamp is disabled while the maneuver is
active.

### Robot assets

- `robot_assets/go1.py`
  Parses the Go1 URDF, assembles simplified mesh parts, caches a standing mesh,
  and creates an articulated visual mesh from 12 joint positions.

### Planning

- `planning/support_cost.py`
  Implements the weighted support-aware objective.
- `planning/select_pose.py`
  evaluates candidates and selects the minimum-cost result.

### Rendering

- `overlays/make_preview_video.py`
  Produces the 2D planning preview from saved trial arrays.
- `overlays/make_chrono_3d_video.py`
  Owns the live simulation/video loop and the Matplotlib renderer.
- `overlays/pyvista_renderer.py`
  Owns the persistent PyVista/VTK scene and native VTK DEM panel.
- `chrono_demo/run_visualizer.py`
  Provides the Chrono Irrlicht interactive experiment.

## Stationary data flow

`run_demo.py` loads all YAML once, then creates an independent Chrono system for
each robot/candidate pair. No terrain state is reused across candidates. This
is important: each candidate is a counterfactual trial from the same initial
bed, not a sequential action on one already-deformed bed.

`run_support_trial` returns arrays in memory. `run_demo.py` then:

1. computes costs across the available candidates;
2. sets the selected candidate on every outcome;
3. saves three height maps per candidate;
4. writes one `outcome.json` per candidate;
5. writes one `summary.json` per robot.

## Traversal state ownership

The traversal loop intentionally has several distinct states:

| State | Owner |
|---|---|
| Trunk planar pose and yaw | `make_chrono_3d_video.py` command integrator |
| Desired feet and joint angles | `TrotGait` |
| Stance-foot vertical settlement | Chrono independent foot bodies |
| SCM node elevations | `SCMTerrain` |
| Contact-adjusted visual leg pose | `contact_adjusted_gait_state` |
| Video camera/materials | selected renderer |

The trunk is kinematic during traversal. Independent feet carry the configured
robot mass during stance but are not connected to the trunk by links or joints.

## Hazard state transition

Hazard mode follows the normal kinematic gait until `find_hazard_strike`
reports a foot/block intersection. The transition is one-way:

```text
kinematic approach
  -> geometric toe strike
  -> freeze/disconnect independent feet
  -> retain locked collision pads opposite the contacted side
  -> unfix low-friction collision-enabled proxy
  -> seed forward/lateral and modest tipping angular velocity
  -> Chrono dynamic skid and rigid-body fall
```

State ownership changes at release:

| State | Before strike | After strike |
|---|---|---|
| Trunk position/rotation | command integrator | Chrono rigid-body dynamics |
| Trunk collision | shapes present, collision disabled | body plus two opposite-side pads enabled |
| Visual joint pose | open-loop gait | last gait pose attached to trunk |
| Independent feet | stance/swing updater | fixed, collision disabled |
| Course geometry | fixed Chrono bodies | fixed Chrono bodies |
| Terrain height map | constant zero array | constant zero array |

The renderer receives the strike flag, leg name, and measured lateral skid but
never computes the fall attitude. It reads the body transform produced by
Chrono after release.

## Difficult-course state flow

Difficult mode remains kinematic for the complete episode:

```text
velocity command -> gait foot targets -> rigid support-height samples
                 -> least-squares support plane -> smoothed roll/pitch/height
                 -> kinematic trunk pose -> world-space terrain foot targets
                 -> inverse trunk transform -> visual IK -> renderer
```

The response time is `0.14 s`, and each roll/pitch component is limited by
`--difficult-max-tilt-deg`. Independent feet still advance in Chrono, but their
pad-edge collision impulses are not fed back into visual IK because that would
mix an unconnected contact body with the kinematic trunk model. Instead, visual
stance-foot Z is constrained directly to each rigid surface.

Rolling mode uses the same support-plane and world-space foot-target path.
`RollingCourse.elevation` generates the initial mesh, then `SCMTerrain` owns the
evolving surface. The live sampled height map drives rendering and DEM values,
while `SCMHeightCourse` queries the same terrain for trunk support and foot
targets. This keeps contact deformation, the 3D scene, and DEM aligned.
`SCMHeightCourse` rejects queries outside the configured pit bounds and returns
the rigid-floor elevation there, matching the actual world ownership at spawn
and after lateral exit.

## Renderer interface

The common video loop computes one frame state containing:

- current and initial height maps;
- robot configuration;
- Chrono trunk body;
- simulation time;
- gait state;
- independent foot bodies;
- traversal and reveal state;
- optional rigid hazard geometry and strike state;
- fixed text describing command, mass, and grid spacing.

Matplotlib consumes those values directly. The VTK path packages them in
`FrameContext` and calls `PyVistaFrameRenderer.render`. The VTK renderer updates
persistent mesh points, scalar arrays, robot vertices, text, and foot markers,
then explicitly renders and reads back an RGB frame.

## Dependency boundaries

- PyVista is imported only when its renderer is constructed.
- Matplotlib remains the default backend and can run without a usable OpenGL
  device.
- Chrono Sensor is not required.
- ROS 2 is not imported anywhere in the current package.
- Genesis and Gaussian packages are not current runtime dependencies.
