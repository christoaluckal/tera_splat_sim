# Roadmap and Extensions

[Documentation index](README.md)

## Completed milestone: rigid hazard and fall video

The reduced-order posterity demonstration is implemented with `--hazard`. It
uses a visible fixed block on a nondeforming center plate, detects an actual
foot/block geometry intersection, releases a low-friction locked-leg proxy with
support retained only opposite the failed side, and lets Chrono rigid contact
evolve a skid and fall. The output records strike, skid, and tilt and labels the
approximation on screen.

Implemented acceptance evidence:

- hazard is visibly localized and parameterized;
- normal SCM traversal remains unchanged and does not release the trunk;
- trigger derives from foot/block geometry rather than video time;
- trunk attitude is evolved by Chrono after release;
- trunk collision keeps the body on the rigid course;
- output reports the `FR` strike at `3.852 s`, `0.332 m` lateral skid, and final
  `90.0 deg` tilt;
- unit tests cover intended-track strike, other-track miss, and clearance miss;
- the rendered video shows approach, strike, fall, and final pose.

The next fidelity step is replacing the geometric trigger, prescribed lateral
velocity, support-pad removal, and tipping velocity with connected leg contact
forces in an articulated robot.

## Priority 2: trustworthy baseline regressions

Before model calibration:

1. Persist full configuration beside every output.
2. Add stationary and traversal numerical regression tests.
3. Record per-frame episode state independently of the renderer.
4. Add grid-convergence trials at 40, 35, 20, and 10 mm.
5. Separate foot-bottom penetration metrics from terrain-height deformation.

## Completed milestone: difficult terrain with completion

The non-failure rigid course is implemented with `--difficult-terrain`. Three
staggered pads generate alternating support heights, the trunk follows a
smoothed bounded support plane, and the robot reaches the far floor. The
reference artifact records `11.8 deg` maximum commanded tilt.

The next fidelity step is making trunk attitude an output of connected robot
dynamics and balance control rather than a kinematic support-plane command.

## Completed milestone: rolling terrain with DEM

`--rolling-terrain --dem-panel` now initializes deformable SCM from a continuous
hills-and-valleys mesh and renders its synchronized `current - initial` DEM.
The floor-clamped reference maneuver reaches `9.4 deg` maximum commanded trunk
tilt and `73.88 mm` maximum subsidence. This milestone improves terrain
variety and deformation visualization, but does not add physical balance
control.

## Completed milestone: forward-turn-forward maneuver

The velocity-command path now supports configurable forward, signed in-place
turn, and second-forward phases. The verified right-turn run ends at
`(0.900, -0.249) m`, yaw `0.0 deg`, while preserving rolling-SCM deformation in
the synchronized DEM. Support-height queries outside the SCM patch use the
rigid floor, preventing spawn-foot penetration. Feedback path tracking and
dynamic balance remain future work.

## Priority 3: connected articulated Go1

A physically valid walking/fall model requires a 13-body floating mechanism
constructed from the included Go1 URDF. The detailed model, motor, contact, and
commissioning contract is in
[Robots and locomotion](robots-and-locomotion.md#articulated-go1-upgrade-specification).
The implementation requires:

- local URDF import because the installed PyChrono bindings have no parser;
- fixed-child mass and inertia consolidation into 13 dynamic bodies;
- `ChBodyAuxRef` link/reference and center-of-mass frames;
- 12 torque motors with URDF angle, velocity, and effort limits;
- calibrated foot collision geometry and robot collision filtering;
- base/IMU state;
- contact sensing;
- body and foot trajectory tracking;
- a balance controller.

The existing visual IK and velocity gait can serve as trajectory references but
must not be mistaken for actuation.

Required gates, in order:

1. fixed-base FK and joint-sign agreement;
2. floating PD stand on rigid flat ground;
3. rigid-ground walking without trunk pose writes;
4. physically generated tilt on uneven rigid terrain;
5. calibrated articulated-foot contact on SCM;
6. ROS 2 effort-interface and external-controller integration.

The minimum acceptance test is a 10-second floating stand with bounded joint
torques, stable contact, finite constraint errors, and no calls that prescribe
trunk position or attitude.

## Priority 4: ROS 2 boundary

Recommended ROS 2 interfaces:

- subscribe to `geometry_msgs/Twist` or stamped equivalent;
- publish robot state, joint state, foot contact, and support outcome;
- separate command conversion from the Chrono simulation clock;
- provide deterministic reset and episode services;
- keep the controller outside rendering code.

ROS 2 packages should wrap the existing motion and simulation APIs rather than
placing ROS logic directly in `make_chrono_3d_video.py`.

## Priority 5: Chrono to MPM forward models

The external proposal in
[`chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md`](archive/chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md)
describes the original extension. The companion `tera_splat` repository now
has:

- an accepted guided 5 mm Chrono A0 oracle with fixed loaded/residual timing;
- an accepted 307,461-particle, 5 mm-particle/n128 Genesis bed;
- candidate-specific initialization and no-action stability gates;
- fixed-time loaded/residual loss and online W&B BayesOpt;
- a confirmed n128 incumbent with independent map-level repeatability.
- a retained-raw replay with aligned isometric surface point clouds, 2D DEM
  error, full-particle PCDs, and 78 sampled rollout PLYs.
- a non-learned Pareto/spatial/recovery/`F`/`Jp` diagnostic and controlled 2x2
  resolution/timestep matrix.

The confirmed `20.433 kPa / 14.727 deg / 0.101895` incumbent has `1.864 mm`
loaded RMSE but `13.678 mm` residual-footprint RMSE. Halving timestep changes
the residual metric by `1.525--2.325 mm`. Same-state traces have now
completed the former localization step: they show wall/ground settling at
`0.5 ms` and free-surface uplift at `0.125 ms`. The next trustworthy
sequence is:

1. correct or ablate one containment/state-preparation numerical mechanism
   while retaining the recorded speed/localization diagnostics and frozen gate;
2. rerun all three same-state preparations and require consistent spatial and
   drift behavior before producing a third response score;
3. if the existing Pareto/recovery mismatch persists after convergence,
   record a Genesis Sand constitutive limitation;
4. only then consider local refinement and held-out loads/locations.

Do not add a learned discrepancy network while the numerical forward model is
not converged.

Do not initialize MPM from an already-deformed Chrono surface and replay the
same load as if that were a model comparison.

### Active Newton alternative branch

[Newton v1.5.1](https://github.com/newton-physics/newton/releases) is viable for
a separate MPM prototype, not a drop-in replacement for the current Genesis
baseline. Its
[implicit MPM solver](https://newton-physics.github.io/newton/stable/api/_generated/newton.solvers.SolverImplicitMPM.html)
supports granular/elasto-plastic particles, and the project includes an
[MPM/rigid two-way coupling example](https://github.com/newton-physics/newton/blob/main/newton/examples/mpm/example_mpm_twoway_coupling.py).
The [general coupling path](https://newton-physics.github.io/newton/stable/concepts/coupling.html)
is experimental, and moving container removal must be tested explicitly for
the [reported kinematic-wall penetration risk](https://github.com/newton-physics/newton/issues/2697).

Reusable across backends:

- the qualified Chrono oracle, action, timing, and valid mask;
- surface-map projection, score definition, diagnostics, and visualization;
- the external manifest and large-output/lightweight-diagnostic separation.

Solver-specific and not reusable as evidence:

- prepared particle state and Genesis `F`/`C`/`Jp` fields;
- material parameter semantics, bounds, incumbent, and BayesOpt observations;
- equilibrium/convergence conclusions and rigid-coupling behavior.

The Newton branch now pins Newton/Warp separately and qualifies a fresh
307,461-particle PIC preparation matrix plus continuous-state, two-way guided
cylinder loading and removal. It writes backend-labelled initial, loaded, and
residual arrays, PLYs, DEMs, masks, traces, metrics, gates, and provenance.
All preparation gates and the full zero-center-penetration mechanics gate pass.
The next work is response convergence across timestep, followed by a fresh
Newton-only calibration/evaluation if response consistency holds. Keep all
backend names and evidence namespaces explicit.

## Priority 6: Gaussian scene deformation

Only after an MPM episode is validated:

1. establish camera-calibrated initial Gaussian geometry;
2. map MPM particles or a deformation field into Gaussian positions;
3. update covariance orientation/scale consistently;
4. define appearance changes separately from mechanics;
5. validate rendered geometry against the source surface;
6. preserve a distinction between synthetic, simulated, and real observations.

## Longer-term evidence ladder

| Stage | Evidence supported |
|---|---|
| Current SCM proxy | qualitative support and deformation demo |
| Reduced-order fall (current) | hazard-triggered rigid trunk fall demonstration |
| Articulated Chrono robot | dynamic stability and controller experiments |
| A0 Genesis bridge (current baseline) | qualified 5 mm oracle, accepted n128 state, stable initialization, and fixed-time response comparison |
| Current Genesis incumbent | loaded-state fit measured; excessive residual recovery remains |
| Newton MPM prototype (active branch) | PIC preparation and one full coupled mechanics response qualified; response timestep convergence and calibration remain |
| Matched MPM backend | calibrated cross-model terrain prediction comparison with backend named |
| Gaussian transfer | physically conditioned visual scene update |
| Real-sand calibration | claims tied to measured material and robot data |
