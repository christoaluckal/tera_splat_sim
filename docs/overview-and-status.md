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
| Cross-model bridge | Genesis MPM in companion `tera_splat` | I/O and BayesOpt implemented; physical calibration awaits Chrono-oracle validation |
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
|-- docs/getting-started.md            concise usage introduction
|-- docs/archive/quick-visual-demo-plan.md
|-- docs/archive/current-system-state-2026-08-13.md
`-- docs/archive/chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md
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

## Cross-model calibration status (updated 2026-08-26)

The companion `tera_splat` repository now has a runnable Chrono-to-Genesis
calibration loop: it transfers the frozen bed geometry, prepares each Genesis
candidate, tests its no-action stability, scores loaded and residual surfaces,
and records online optimization results. That operational status concerns the
data path and optimizer, not the physical validity of the Chrono target.

The completed `A0_cal_full10mm` studies use a legacy free-centered 10 mm
reference. A fresh replay matches its stored maps within `0.00076 mm`, so it
is not stale data or an obsolete Chrono build. It is retired from future
calibration because it has an incomplete residual-time contract and an
unqualified free-load protocol. Keep its results as I/O/optimizer evidence but
do not seed or mix them into the next W&B study.

The active R&D protocol is a 1.5 kg vertically guided cylinder at `(0, +5) mm`
on a 0.6 m, 10 mm SCM screen. This offset guided case has the cleanest observed
cross-section; it is an iteration-speed choice, not a physical parameter. The
same guided protocol at 5 mm is reserved for final high-fidelity validation.
Both current compact cases end at a fixed loading timeout, so neither is an
exportable oracle yet. The next code change is a recorded speed-and-hold
loading convergence gate, followed by fixed-duration residual recovery. Once a
fresh 10 mm episode passes that contract, the existing Genesis bridge and
BayesOpt logic run unchanged with a new isolated W&B study.

## SCM oracle runtime (2026-08-25)

The `chrono_splat` Conda environment now activates a separately built,
headless Chrono `10.0.0` Vehicle/Python binding from
`/data/christoa/Chrono/build/projectchrono-10.0.0-vehicle-py310`. The source
is pinned at commit `9faf13dd8f1128dd75ed233a9627027b0422c3f7` and was built
against the `chrono_splat` Python 3.10 interpreter with core, Vehicle, Vehicle
Models, and Python bindings enabled. The Conda package remains in place, but
the activation hook gives the full binding precedence, so SCM instrumentation
continues to run through `chrono_splat` without changing the Genesis package
stack.

A direct smoke test imported `pychrono.vehicle`, constructed a Bullet-backed
`SCMTerrain`, initialized a 10 mm patch, and sampled its height successfully.
For reproducible oracle-resolution studies,
`run_cylinder_episode.py --scm-grid-spacing-m <spacing>` overrides only that
episode; it does not edit the shared terrain configuration. The completed time-captured 5 mm guided episode is at
`/data/christoa/Chrono/tera_splat_sim/validity_experiment/chrono_episodes/A0_oracle_mass1p5kg_guided_5mm`.
It is final-validation evidence only until the new convergence gate records a
stable loading state.

## SCM translation/phase check correction (2026-08-26)

The earlier grid-lock conclusion was invalid because its centroid calculation
included the excluded SCM boundary ring. With `valid_heightmap_mask.npy`, the
early deformation moves with the shifted cylinder. The compact test indicates
millimetre-scale coarse-grid phase sensitivity, not a fixed world-coordinate
feature. Guided and finer-resolution validation remain appropriate. The
legacy target is retired for its timing/protocol contract, not because this
phase test alone invalidates a 10 mm SCM surface.

Corrected values and artifact paths are in
[`tera_splat/docs/chrono-oracle-diagnostics.md`](../../tera_splat/docs/chrono-oracle-diagnostics.md).
The next-run decision and legacy-observation policy are in
[`tera_splat/docs/chrono-oracle-run-contract.md`](../../tera_splat/docs/chrono-oracle-run-contract.md).
