# Chrono to Genesis MPM to Gaussian Terrain Pipeline

> **Document status:** External extension proposal, not an implemented pipeline.
> Current implementation and roadmap status are indexed at
> [`docs/README.md`](docs/README.md).

Snapshot date: 2026-08-12

## 1. Executive summary

The current Chrono prototype is sufficient to begin the terrain-deformation
pipeline. It can act as a deterministic synthetic support oracle that produces
initial, loaded, and residual SCM heightfields under a known robot or rigid-body
action. The next milestone is not articulated walking. It is a common episode
interface that allows the same initial terrain and action to be evaluated in
Chrono SCM and Genesis MPM, followed by deformation of a Gaussian terrain map
using the MPM displacement and deformation-gradient fields.

The intended pipeline is:

```text
known terrain + robot/action
             |
             v
     Chrono SCM rollout
             |
             | synthetic target surfaces and support outcomes
             v
 Genesis MPM identification and rollout
             |
             | displacement u and deformation gradient F
             v
   Gaussian mean/covariance update
             |
             v
   RGB/depth rendering and planning
```

Chrono SCM is an intermediate synthetic oracle, not physical ground truth. Real
D455 measurements of a controlled sand bed remain the final calibration and
evaluation reference.

Historical Chrono outputs must not be used as Genesis calibration targets. The
SCM friction-angle unit bug and always-enabled smoke-grid path were corrected on
2026-08-12. A corrected nominal-mass coarse-grid trial and comparison video now
exist, but a deterministic full-resolution baseline and episode manifest are
still required before Genesis calibration.

## 2. Relationship to the current system

This document continues `CURRENT_SYSTEM_STATE.md`. That document describes the
implemented Chrono quick-support demo, including its current physics proxy,
rendering paths, outputs, limitations, and known correctness issues.

Implemented before this transition:

- a rigid perimeter floor and central Chrono SCM patch;
- locked four-foot Go1 and Spot support proxies;
- a real Go1 visual mesh assembled in a fixed stance;
- deterministic initial, loaded, and residual terrain heightmaps;
- body sinkage and tilt measurements;
- a support-aware two-candidate planner;
- reliable headless Matplotlib video generation.

Not required before starting this transition:

- articulated Go1 or Spot dynamics;
- a locomotion controller;
- a closed-loop or dynamically coupled gait controller;
- ROS 2 integration;
- a Spot visual mesh;
- working Irrlicht frame capture;
- Chrono Sensor support.

The locked support proxy is adequate for the first controlled deformation
episodes because the research question at this stage concerns terrain response
under known contact geometry and loading, not gait-controller fidelity.

## 3. Component responsibilities

| Component | Responsibility | Primary outputs |
|---|---|---|
| Chrono SCM | Fast robot-conditioned support oracle | Heightfields, sinkage, tilt, robot pose, contact configuration |
| Genesis MPM | Volumetric material response and counterfactual prediction | Particle position, displacement, velocity, deformation gradient, predicted surface |
| Gaussian map | Persistent visual scene representation | Gaussian means, covariances, opacity, spherical harmonics, rendered RGB/depth |
| External renderer | Camera-calibrated synthetic observations | RGB, metric depth, segmentation, intrinsics, extrinsics |
| Real D455 experiment | Final physical calibration and held-out evaluation | Metric before/loaded/residual surfaces in the fixed `bed` frame |
| Planner | Robot-conditioned action or support selection | Candidate cost and selected action |

The components must remain modular. Chrono does not need to render the final
Gaussian scene, Genesis does not need to become the robot simulator, and the
Gaussian representation does not need to replace the volumetric MPM state.

## 4. Important interpretation boundaries

Chrono SCM models terrain through pressure-sinkage and shear relationships on a
persistent heightfield. It can produce rutting, settlement, and optional
bulldozed berms. It does not simulate individual grains or expose the same
internal state as MPM.

Genesis MPM represents a deformable material volume with particles and grid
transfers. Its internal state may include particle positions, velocities,
affine velocity terms, deformation gradients, and plastic state. A Chrono SCM
heightfield alone does not determine these quantities.

Therefore:

- Chrono can supply target surface behavior for a controlled one-action trial.
- Chrono cannot provide a complete Genesis checkpoint for a sequential trial.
- Matching Genesis to Chrono creates an SCM-matched surrogate, not a validated
  real-sand model.
- Real-sand data is required for the final material and sim-to-real claims.

## 5. Chrono baseline correction status

### 5.1 SCM friction-angle unit correction

The YAML field `mohr_friction_deg` contains `28.0`. The builder now passes this
value directly to `SCMTerrain.SetSoilParameters`, matching the installed
PyChrono 8 header, which specifies degrees.

Implemented correction:

```python
# Incorrect for the current PyChrono API.
friction = math.radians(config["mohr_friction_deg"])

# Required.
friction = float(config["mohr_friction_deg"])
```

Existing results produced with approximately `0.489` instead of `28.0` must not
be used as calibrated sand behavior.

### 5.2 Video smoke/full-resolution correction

The video path now defaults to the configured 10 mm terrain and exposes explicit
mutually exclusive resolution options:

Implemented CLI:

```text
--smoke       use coarse grid and shortened rollout
--full-res    use configured grid and rollout settings
```

The corrected nominal comparison deliberately uses `--smoke` for practical
runtime and labels the resulting 35 mm grid in-frame. The full-resolution path
is reachable but has not yet produced a retained baseline video.

### 5.3 Baseline sequence

After the two corrections:

1. Run a nominal-mass single-load trial without bulldozing.
2. Verify finite heightfields and stable rigid-body settling.
3. Verify that increased mass produces non-decreasing sinkage under otherwise
   identical conditions.
4. Record initial, loaded, and residual heightfields.
5. Enable bulldozing through configuration as a separate experiment.
6. Regenerate the current-code Go1/Spot candidate comparison.
7. Store the full configuration and code revision with every episode.

Bulldozing must not be introduced while the baseline material parameters or
friction units are still uncertain.

## 6. Canonical episode definition

Every Chrono and Genesis comparison must refer to the same episode definition.
An episode contains:

- an initial surface in a metric world frame;
- terrain bounds and base depth;
- rigid contact geometry;
- body mass and inertia;
- initial body pose and velocity;
- gravity;
- action trajectory or release condition;
- loaded-state sample time;
- removal time and residual-state sample time;
- solver configuration and material parameters.

The first supported action should be either:

- the existing locked four-foot support proxy released under gravity; or
- a single known cylinder placed under gravity.

Both are valid. The single cylinder is easier for initial solver-to-solver
matching, while the existing Go1 proxy provides the direct planning
demonstration.

### 6.1 Proposed episode directory

```text
episode/
|-- manifest.yaml
|-- action.json
|-- contact_geometry.json
|-- robot_poses.csv
|-- camera_intrinsics.json
|-- initial_heightmap_m.npy
|-- loaded_heightmap_m.npy
|-- residual_heightmap_m.npy
|-- valid_heightmap_mask.npy
`-- metrics.json
```

### 6.2 Required manifest fields

```yaml
schema_version: 1
episode_id: <unique identifier>
source: chrono_scm
frame_id: bed
units:
  length: m
  mass: kg
  time: s

terrain:
  bounds_xy_m: [xmin, xmax, ymin, ymax]
  grid_spacing_m: <value>
  surface_elevation_m: <value>
  base_depth_m: <positive depth below surface>
  bulldozing_enabled: <true or false>
  scm_parameters: <complete parameter mapping>

action:
  type: gravity_release
  geometry_id: <identifier>
  mass_kg: <value>
  initial_pose_world: <position and quaternion>
  release_time_s: <value>
  removal_time_s: <value>

sampling:
  initial_time_s: <value>
  loaded_time_s: <value>
  residual_time_s: <value>

solver:
  pychrono_version: 8.0.0
  timestep_s: <value>
  settle_thresholds: <mapping>
  code_revision: <git revision or explicit unknown>
```

Every array must state its world-frame convention, axis order, origin, spacing,
units, and invalid-value convention. Do not infer these later from file shape.

## 7. Chrono to Genesis comparison protocol

Let:

- \(H_0\) be the initial terrain surface;
- \(H_L\) be the surface under the applied load;
- \(H_R\) be the residual surface after load removal;
- \(A\) be the complete rigid-body action;
- \(\theta\) be the Genesis material parameters.

Chrono produces the target episode:

\[
H_0, A
\xrightarrow{\mathrm{Chrono\ SCM}}
H_L^C, H_R^C, y^C,
\]

where \(y^C\) contains support outcomes such as foot sinkage, body tilt, and
center-of-mass displacement.

Genesis runs independently from the same initial condition:

\[
H_0, A, \theta
\xrightarrow{\mathrm{Genesis\ MPM}}
H_L^M, H_R^M, y^M.
\]

The Genesis parameters are selected to reduce the disagreement between the two
rollouts.

### 7.1 Invalid double-deformation shortcut

Do not initialize Genesis from \(H_L^C\) and apply the same action again. This
would treat the already-deformed Chrono surface as an undeformed Genesis state
and would apply the deformation twice.

The correct comparison initializes both systems from \(H_0\) and applies the
same action once.

### 7.2 Sequential actions

The surface \(H_L^C\) is not a complete state for continuing the episode in
Genesis. It does not contain internal stress, plastic strain, compaction,
particle velocity, deformation gradient, or other hidden MPM state.

Initial pipeline restriction:

- treat every action as an independent rollout from a reset bed;
- retain Genesis checkpoints only for Genesis-generated sequential actions;
- do not claim Chrono-to-Genesis sequential state transfer from height alone.

Sequential sim-to-sim transfer is a separate research problem.

## 8. Genesis MPM bed construction

Chrono SCM produces a 2.5D heightfield, while MPM requires a volumetric material
domain. Construct the initial MPM bed by filling particles below the initial
surface:

\[
z_{\mathrm{base}} \leq z \leq H_0(x,y).
\]

Recommended construction:

1. Resample \(H_0\) onto the selected Genesis particle or cell spacing.
2. For each horizontal sample, populate particles from the fixed base to the
   surface height.
3. Fix or constrain the bed base.
4. Add containment conditions at the physical tray or region boundaries.
5. Allow the material to settle under gravity before applying the action.
6. Save the settled particle state as the reusable Genesis initial checkpoint.
7. Apply the rigid contact action using the same geometry, mass, pose, gravity,
   timing, and removal definition as the Chrono episode.

The settled Genesis checkpoint, rather than a newly generated loose particle
cloud, should be reused across parameter evaluations whenever the evaluated
parameters permit it. If settling depends materially on the candidate
parameters, settling becomes part of each evaluated rollout.

### 8.1 Surface extraction from MPM

The Genesis output must be converted to a heightfield in the same `bed` frame
as Chrono. The extraction method must be deterministic and stored in the
manifest.

Possible extraction methods include:

- highest occupied particle per XY bin;
- density-threshold isosurface followed by vertical sampling;
- top-layer particle interpolation onto the Chrono grid.

For the first prototype, use the highest valid particle per XY bin followed by
controlled hole filling. Keep the raw particle output so the surface extraction
can be replaced without rerunning every simulation.

## 9. Material identification objective

The initial fitting problem should use a small number of effective material
parameters. Do not expose every available Genesis parameter simultaneously.

A generic objective is:

\[
\begin{aligned}
\mathcal{L}(\theta) ={}&
w_h\,\operatorname{RMSE}\!\left(H_L^M,H_L^C\right)
+w_r\,\operatorname{RMSE}\!\left(H_R^M,H_R^C\right) \\
&+w_s\left|s^M-s^C\right|
+w_t\left|\alpha^M-\alpha^C\right|
+w_v\left|V^M-V^C\right|,
\end{aligned}
\]

where:

- \(s\) is contact or foot sinkage;
- \(\alpha\) is maximum absolute body tilt;
- \(V\) is displaced volume over a shared valid region.

Additional optional terms:

- radial deformation-profile error around each contact;
- deformation-radius error;
- maximum-depth error;
- berm-height or positive-volume error when bulldozing is enabled;
- center-of-mass height-change error.

All surface terms must use a shared valid mask and must exclude known SCM
boundary artifacts.

### 9.1 Parameter fitting stages

Use staged identification:

1. Match maximum and mean sinkage under one simple centered load.
2. Match the loaded deformation profile and radius.
3. Match the residual surface after removal.
4. Evaluate a second mass without refitting.
5. Evaluate a shifted contact location without refitting.
6. Transfer the fitted parameters to the locked four-foot proxy.

The first fit may produce an effective SCM-equivalent Genesis material. It must
not be described as uniquely identifying physical sand parameters.

## 10. MPM to Gaussian deformation transfer

The Gaussian map remains the visible scene representation. Genesis particles
remain the volumetric physics representation. A spatial transfer connects them.

For Gaussian \(g\), interpolate the displacement of neighboring MPM particles:

\[
\boldsymbol{u}_g
=
\sum_{p\in\mathcal{N}(g)}
w_{gp}
\left(\boldsymbol{x}'_p-\boldsymbol{x}_p\right),
\qquad
\sum_p w_{gp}=1.
\]

Update the Gaussian mean:

\[
\boldsymbol{\mu}'_g
=
\boldsymbol{\mu}_g+\boldsymbol{u}_g.
\]

Interpolate the deformation gradient:

\[
\boldsymbol{F}_g
=
\sum_{p\in\mathcal{N}(g)}
w_{gp}\boldsymbol{F}_p.
\]

Update the Gaussian covariance:

\[
\boldsymbol{\Sigma}'_g
=
\boldsymbol{F}_g
\boldsymbol{\Sigma}_g
\boldsymbol{F}_g^{\mathsf T}.
\]

Use the polar decomposition

\[
\boldsymbol{F}_g=\boldsymbol{R}_g\boldsymbol{S}_g
\]

to obtain the local rotation \(\boldsymbol{R}_g\) used to rotate the Gaussian's
view-dependent appearance basis.

### 10.1 Initial attribute policy

For the first implementation:

- update Gaussian means using MPM displacement;
- update Gaussian covariances using the deformation gradient;
- rotate spherical-harmonic orientation using the rotational part of
  \(\boldsymbol{F}_g\);
- retain opacity;
- retain spherical-harmonic coefficients apart from orientation handling;
- update only terrain Gaussians within the MPM region;
- leave robot, tray, tags, floor, and background Gaussians unchanged.

Appearance changes caused by newly exposed grains, occlusion changes, or
surface abrasion are outside the first milestone.

### 10.2 Surface and internal particles

Do not use only the reconstructed surface Gaussians as the entire MPM material
volume. Terrain support requires subsurface mass.

Use:

- surface Gaussians for rendering;
- volumetric MPM particles for physics;
- a stored neighborhood or interpolation mapping between them.

Internal MPM particles do not need to be visible unless deformation exposes a
region that was previously beneath the surface. Handling newly exposed
appearance is a later extension.

## 11. Use of PhysGaussian

The reference paper is `2311.12198v3.pdf`, PhysGaussian: Physics-Integrated 3D
Gaussians for Generative Dynamics.

Relevant ideas to reuse:

- treat Gaussian means as points advected by a continuum displacement field;
- evolve covariance with the deformation gradient;
- rotate spherical-harmonic viewing directions with local rotation;
- use Drucker-Prager plasticity for granular material behavior;
- maintain a unified visible representation after physics-based deformation.

Ideas that do not directly solve the current terrain problem:

- PhysGaussian's opacity-based internal filling targets closed or enclosed
  reconstructed objects. A terrain bed is an open volume with a known surface,
  base, and lateral bounds, so direct column filling is more appropriate.
- The paper manually specifies physical parameters and does not provide the
  required Chrono-to-Genesis or real-to-Genesis calibration procedure.
- Its examples demonstrate generative dynamics and rendering quality, not
  validated robot-terrain support prediction.
- Its custom MPM implementation must not be assumed to have the same material
  state, contact behavior, or parameter meanings as Genesis MPM.

The current project should reuse the Gaussian kinematic update, not reproduce
the paper's complete MPM implementation.

## 12. Camera-calibrated visual generation

The existing Matplotlib video is valid for a quick visual demonstration of
Chrono-computed deformation. It is not a camera-calibrated RGB-D dataset for
Gaussian reconstruction.

Chrono Sensor is unavailable in the current PyChrono installation and Irrlicht
headless screenshots are black. The proposed synthetic capture path is
therefore:

1. Sample the Chrono SCM heightfield at the required state or video frame.
2. Convert the regular heightfield into a triangle mesh.
3. Export the robot visual mesh with its Chrono rigid-body transform.
4. Export the rigid floor, tray, target, and other scene geometry.
5. Render RGB, metric depth, and segmentation using Blender in headless mode.
6. Store exact camera intrinsics and world-to-camera extrinsics.
7. Write a COLMAP-compatible camera and image dataset.

### 12.1 Gaussian training policy

Train the Gaussian scene from the initial undeformed state \(S_0\). Later
states should be generated by deforming this map with Genesis MPM.

Training an independent Gaussian map for every state may be useful as a visual
upper bound or evaluation reference, but it is not the intended predictive
pipeline.

### 12.2 Visual demonstration levels

| Level | Physics | Rendering | Intended use |
|---|---|---|---|
| Current | Chrono SCM | Matplotlib | Quick proof of deformation and planning |
| Synthetic dataset | Chrono SCM | Blender RGB-D/segmentation | Train and verify a metric Gaussian scene |
| Predictive prototype | Genesis MPM | Deformed Gaussian map | Counterfactual visual prediction |
| Final evaluation | Real sand and calibrated Genesis | D455 observations and Gaussian rendering | Sim-to-real validation |

## 13. ROS 2 integration boundary

ROS 2 is not required for the first offline solver-to-solver comparison. The
episode schema should nevertheless use metric frames and timestamped poses so
that it can later be connected to ROS 2.

The eventual ROS 2 boundary is:

```text
sensor observations and robot state
                 |
                 v
          Gaussian terrain map
                 |
          candidate action query
                 |
                 v
      Genesis support prediction
                 |
                 v
        ROS 2 planner/controller
```

Chrono can later execute planned robot motion or replay ROS 2 trajectories, but
the current Chrono environment does not yet contain ROS, sensors, or an
articulated controller.

## 14. Proposed implementation modules

The following names are proposed interfaces, not a statement that the files
already exist:

```text
quick_support_demo/
|-- interchange/
|   |-- episode_schema.py
|   |-- export_chrono_episode.py
|   |-- validate_episode.py
|   `-- heightfield_mesh.py
|-- rendering/
|   |-- export_blender_scene.py
|   |-- render_rgbd.py
|   `-- write_colmap.py
|-- genesis_bridge/
|   |-- build_mpm_bed.py
|   |-- replay_action.py
|   |-- extract_surface.py
|   |-- calibrate_material.py
|   `-- export_particle_state.py
`-- gaussian_bridge/
    |-- build_particle_mapping.py
    |-- deform_gaussians.py
    `-- validate_render.py
```

The interfaces should depend on the episode schema rather than directly
importing one another's simulator-specific objects.

## 15. Implementation phases

### Phase A: trustworthy Chrono source episode

1. Completed: correct the SCM friction-angle units.
2. Completed: correct smoke/full-resolution selection.
3. Completed: add runtime checks for finite values and expected heightmap dimensions.
4. Completed at smoke resolution: generate one nominal centered-load trial.
5. Generate a second-mass episode.
6. Save full manifests and raw heightfields.

Exit condition: two deterministic episodes can be regenerated from stored
configuration, and increased mass does not produce less sinkage under the same
conditions.

### Phase B: common terrain and action interface

1. Implement the episode schema.
2. Export Chrono data into the schema.
3. Validate units, axes, grid origin, and array ordering.
4. Add a command that visualizes initial, loaded, residual, and difference
   heightfields from one episode.

Exit condition: no simulator-specific assumptions are needed to read and
compare the saved surfaces.

### Phase C: Genesis replay

1. Fill a volumetric MPM bed from \(H_0\).
2. Settle and checkpoint the bed.
3. Recreate the Chrono rigid contact geometry and action.
4. Extract Genesis loaded and residual surfaces.
5. Compare Genesis and Chrono using the shared metrics.

Exit condition: one command produces aligned Chrono and Genesis surface and
support metrics for the same episode.

### Phase D: effective material identification

1. Select a small initial Genesis parameter set.
2. Fit against one centered-load episode.
3. Evaluate without refitting on the second mass.
4. Evaluate without refitting at a shifted contact location.
5. Record optimization history and final parameters.

Exit condition: the held-out action is evaluated with fixed parameters and the
result is reported separately from the fitting episode.

### Phase E: synthetic Gaussian scene

1. Render a camera-calibrated initial scene in Blender.
2. Write the corresponding COLMAP dataset.
3. Train an initial Gaussian map.
4. Identify terrain Gaussians and preserve rigid-scene Gaussians.
5. Verify metric alignment between the Gaussian map and the `bed` frame.

Exit condition: the undeformed Gaussian terrain aligns with \(H_0\) and renders
from held-out cameras.

### Phase F: MPM-conditioned Gaussian deformation

1. Export Genesis particle displacement and deformation gradient.
2. Build the MPM-particle-to-Gaussian mapping.
3. Update Gaussian means.
4. Update covariance and orientation.
5. Render the predicted loaded and residual states.
6. Compare rendered depth against the simulator target surface.

Exit condition: one initial Gaussian scene can be transformed into a distinct
deformed scene without retraining.

### Phase G: real-sand replacement

1. Capture real before, loaded, and post-removal D455 states.
2. Express all observations in the fixed AprilTag `bed` frame.
3. Replace Chrono target heightfields with real measured surfaces.
4. Refit or evaluate Genesis parameters.
5. Preserve Chrono as a planning and synthetic-data baseline.

Exit condition: the same evaluation code accepts either Chrono or real targets,
with the source clearly recorded in the episode manifest.

## 16. Acceptance criteria

| Criterion | Required result |
|---|---|
| Correct Chrono configuration | Friction uses degrees and full-resolution mode is reachable |
| Deterministic episode export | Repeated run reproduces heightmaps within defined tolerance |
| Shared coordinate system | Chrono, Genesis, renderer, and Gaussian map agree in meters and in the `bed` frame |
| Volumetric MPM initialization | Particles fill the bed beneath \(H_0\), not only its surface |
| Same-action comparison | Chrono and Genesis use matching geometry, mass, pose, gravity, and timing |
| Held-out evaluation | At least one mass or location is evaluated without refitting |
| Gaussian deformation | A single initial map produces a loaded state without retraining |
| Covariance evolution | Gaussian covariance responds to \(\boldsymbol{F}\), not only mean translation |
| Rigid-scene preservation | Robot, tray, floor, and background Gaussians remain rigid |
| Honest provenance | Every result is labeled Chrono-targeted, synthetic, or real-data validated |

## 17. Known invalid shortcuts

Do not:

- treat the current friction-bug outputs as calibrated sand;
- describe Chrono SCM as grain-resolved simulation;
- initialize Genesis from a loaded Chrono surface and repeat the same action;
- infer a complete MPM state from a surface heightfield;
- simulate only the visible Gaussian surface without subsurface material;
- retrain a new Gaussian scene for every predicted state and call it physical
  state propagation;
- interpret separate Chrono candidates as sequential interactions with one
  persistent bed;
- call a synthetic Chrono-to-Genesis match sim-to-real validation;
- prioritize articulated walking before the deformation interface is working;
- use the Matplotlib video as calibrated RGB-D training data;
- hide vertical exaggeration in the final deformation visualization.

## 18. Claims supported at each stage

### Current Chrono state

Supported claim:

> A robot-conditioned rigid support proxy in Chrono SCM produces measurable
> persistent heightfield deformation and support outcomes that can influence a
> shared candidate-selection objective.

### After Chrono-to-Genesis matching

Supported claim:

> A Genesis MPM surrogate can be identified to reproduce selected surface and
> support outcomes of a controlled Chrono SCM oracle and evaluated on held-out
> synthetic actions.

This is a synthetic cross-model transfer claim, not a real-sand validation
claim.

### After Gaussian deformation

Supported claim:

> A persistent Gaussian terrain representation can be updated from MPM
> displacement and deformation-gradient fields to render action-conditioned
> counterfactual terrain states without retraining the scene.

### After real-sand evaluation

Potential supported claim:

> A material-calibrated MPM model and physics-conditioned Gaussian terrain map
> predict the geometry and appearance of controlled real terrain deformation
> under held-out loads or contact locations.

The final wording depends on measured generalization performance.

## 19. Recommended next action

The next implementation task is to finish Phase A and then begin Phase B:

1. generate and retain a full-resolution centered nominal-mass episode;
2. generate a second-mass episode and verify monotonic sinkage;
3. add a deterministic repeated-run tolerance test;
4. export the nominal episode through the canonical episode schema;
5. validate its heightfields and coordinate metadata.

Do not begin material optimization or tune Genesis against the existing Chrono
outputs before these steps are complete.

## 20. Agent handoff checklist

Before modifying the pipeline, an agent should verify:

- `CURRENT_SYSTEM_STATE.md` has been read;
- the current PyChrono version is still 8.0.0;
- friction is passed to SCM in the units expected by that installed version;
- smoke and full-resolution settings are distinguishable in saved manifests;
- every comparison begins from the same \(H_0\);
- action geometry, mass, pose, gravity, and timing are identical across solvers;
- SCM boundary artifacts are excluded with an explicit valid mask;
- Genesis particles fill a volume beneath the terrain surface;
- the Gaussian update uses both displacement and deformation gradient;
- output claims identify whether the target is Chrono or real data;
- no controller or articulated-walking fidelity is implied by the locked proxy.

If any of these conditions are unknown, record the uncertainty in the episode
manifest or experiment report instead of silently selecting a value.
