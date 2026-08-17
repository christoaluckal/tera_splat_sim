# Overview and Status

[Documentation index](README.md)

## Objective

The original objective is a visual demonstration that terrain support depends
on both terrain and robot properties. The same deformable sand candidate can
produce different support outcomes for robots with different mass, stance, and
foot geometry. A support-aware planner compares that candidate with a rigid
bypass candidate.

The repository now also contains a Go1 traversal sequence. A planar command
`(vx, vy, wz)` drives an open-loop trot visual and four independent Chrono foot
contact bodies across the SCM pit. This was added to make sequential footprints
and terrain deformation visible. It is separate from the original stationary
candidate-selection experiment.

## Implemented workflows

### Stationary support planning

For each selected robot and candidate:

1. Build a new Chrono SMC system.
2. Add four rigid floor boxes around the pit opening.
3. Add and initialize a Chrono SCM terrain patch.
4. Sample the initial height map.
5. Add a rigid four-foot support proxy at the candidate pose.
6. Simulate until settled or until the configured time limit.
7. Measure foot sinkage, body attitude, COM height change, and loaded terrain.
8. Remove the robot and simulate residual terrain settling.
9. Save outcomes and compute the shared planning cost.

### Velocity-command traversal

1. Build the same world and SCM pit.
2. Create a kinematic Go1 trunk proxy at the near edge.
3. Convert `(vx, vy, wz)` into open-loop trot foot targets.
4. Solve 12 visual joint angles with analytical inverse kinematics.
5. Move four independent contact feet through stance and swing phases.
6. Advance Chrono and SCM at each physics step.
7. Sample the SCM surface at each requested video frame.
8. Render the moving robot, terrain, deformation metrics, and optional DEM.

### Rigid hazard traversal

1. Replace the SCM pit with a fixed rigid center plate.
2. Place a fixed rigid block at a configurable lateral x offset.
3. Run the same open-loop Go1 gait toward the far side.
4. Detect a geometric intersection between a foot and the block.
5. Stop kinematic trunk updates and disable the independent gait feet.
6. Release a low-friction rigid proxy with locked collision pads only on the
   side opposite the contacted hazard.
7. Seed forward/lateral velocity and a modest one-time tipping angular velocity.
8. Let Chrono gravity and rigid contact evolve the skid and trunk fall.
9. Render the strike leg, lateral skid, trunk tilt, and approximation label.

### Rigid difficult-terrain traversal

1. Replace SCM with a fixed center plate and three staggered raised pads.
2. Run the same open-loop velocity gait without a failure trigger.
3. Query rigid support height below each commanded foot location.
4. Fit a plane through the four support-height samples.
5. Smooth and clamp the resulting trunk roll, pitch, and elevation.
6. Continue to the far rigid floor and hold the completed pose.
7. Recompute visual foot Z from the local rigid surface before IK.
8. Render current tilt and the kinematic terrain-following label.

### Deformable rolling-terrain DEM traversal

1. Initialize SCM from a triangle-mesh heightfield.
2. Use the live SCM surface for visual geometry, DEM difference,
   support-plane fitting, and visual foot targets.
3. Traverse a near hill, center valley, and far hill.
4. Render the perspective robot view and `current - initial` DEM side by side.
5. Preserve the footprints after the robot reaches the far floor.

### Forward-turn-forward maneuver

1. Hold a `0.8 s` traversal warmup.
2. Command forward speed until the first configured distance is reached.
3. Command zero translation and signed yaw rate for the configured angle.
4. Command forward speed in the new heading for the second distance.
5. Hold the completed pose and render the current maneuver phase.

## Capability matrix

| Area | Capability | Status and boundary |
|---|---|---|
| World | 3 m square floor and central pit | Implemented |
| Terrain | Chrono SCM deformable patch | Implemented |
| Terrain | Nondeforming rigid center course and offset block | Implemented |
| Terrain | Nondeforming staggered difficult-course pads | Implemented |
| Terrain | Deformable SCM hills-and-valleys heightfield | Implemented |
| Terrain | Calibrated real granular sand | Not implemented |
| Robot | Go1 URDF-derived appearance | Implemented |
| Robot | Spot detailed visual asset | Not implemented |
| Contact | Stationary rigid four-foot proxy | Implemented |
| Contact | Traversal with independent feet | Implemented approximation |
| Motion | Body velocity command | Implemented |
| Motion | Open-loop trot and IK | Implemented |
| Motion | Forward-turn-forward phase scheduler | Implemented |
| Motion | Hazard-triggered reduced-order rigid-trunk fall | Implemented approximation |
| Motion | Support-plane trunk tilt through difficult course | Implemented approximation |
| Motion | Terrain-aware trunk/foot targets on rolling course | Implemented approximation |
| Motion | Physically connected leg joints | Not implemented |
| Control | Balance, state estimation, recovery | Not implemented |
| Planning | Two-candidate weighted objective | Implemented |
| Output | JSON outcomes and NumPy height maps | Implemented |
| Output | Matplotlib videos | Implemented |
| Output | PyVista/VTK PBR videos | Implemented for Go1 |
| Output | Lateral foot-clearance camera | Implemented |
| Output | Signed DEM difference | Implemented |
| Output | Mesh-initialized SCM DEM difference | Implemented |
| Output | Calibrated final-state RGB-D orbit dataset | Implemented for Go1/PyVista |
| Integration | ROS 2 | Not implemented |
| Extension | Genesis MPM | Proposed only |
| Extension | Gaussian deformation transfer | Proposed only |

## Repository layout

```text
Chronos/
|-- docs/                              canonical documentation
|-- quick_support_demo/
|   |-- assets/go1/                    URDF, source meshes, simplified meshes
|   |-- chrono_demo/                   world, SCM, contacts, trials
|   |-- configs/                       YAML configuration
|   |-- motion/                        velocity command, gait, IK
|   |-- overlays/                      preview and 3D video backends
|   |-- planning/                      support cost and candidate selection
|   |-- robot_assets/                  Go1 visual assembly
|   |-- tests/                         unit tests
|   `-- outputs/                       generated trials, frames, and videos
|-- run_demo.py                        stationary experiment entry point
|-- README.md                          concise usage introduction
|-- QUICK_VISUAL_DEMO_PLAN.md          original plan
|-- CURRENT_SYSTEM_STATE.md            historical monolithic state
`-- CHRONO_TO_MPM_GAUSSIAN_PIPELINE.md extension proposal
```

## What the current demo demonstrates

The stationary workflow demonstrates that:

- the SCM bed deforms under robot-conditioned loading;
- robot mass and contact geometry affect the measured support outcome;
- the same cost function can compare a deformable close pose and a rigid
  farther pose;
- deformation can persist after unloading.

The traversal workflow demonstrates that:

- a body-frame velocity command can drive an open-loop Go1 trot visualization;
- discrete stance feet can generate a sequence of SCM footprints;
- a signed DEM can be synchronized with the 3D loading scene;
- the same simulation state can be rendered through Matplotlib or VTK.

The rigid hazard workflow demonstrates that an offset nondeforming obstacle can
intersect a commanded foot path and trigger a visible lateral skid followed by
a simulated rigid-body fall. It does not demonstrate force propagation through
articulated legs; release velocity and support loss are reduced-order inputs.

The difficult-terrain workflow demonstrates a complete crossing with visible,
alternating trunk attitude over known rigid support heights. Its trunk attitude
is commanded from a fitted support plane rather than produced by a controller.

## What it does not demonstrate

The current repository does not demonstrate:

- dynamic quadruped balance;
- actuator torques or joint-level motor control;
- stability margins derived from a connected multibody robot;
- slip-calibrated locomotion over real sand;
- a ROS 2 locomotion stack;
- MPM agreement with SCM;
- physically updated Gaussian scene appearance;
- an articulated, contact-force-induced quadruped fall during traversal.

See [limitations and supported claims](limitations-and-claims.md) before using
the outputs in a report or presentation.
