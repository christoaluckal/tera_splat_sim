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

## Priority 5: Chrono to Genesis MPM

The external proposal in
[`chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md`](archive/chrono-to-mpm-gaussian-pipeline-proposal-2026-08-12.md)
describes the original extension. Its first three interface steps now exist in
the companion `tera_splat` repository: a full-resolution 10 mm Chrono A0
episode, an accepted volumetric CPIC Genesis bed restored from complete MPM
state, and common-grid initial/loaded/residual maps. The Genesis response is
not yet calibrated, so the next trustworthy sequence is:

1. Freeze that A0 action, prepared state, and common 10 mm grid.
2. Fit effective MPM parameters in stages against the loaded/residual maps.
3. Validate on held-out loads and locations.

Do not initialize MPM from an already-deformed Chrono surface and replay the
same load as if that were a model comparison.

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
| A0 Genesis bridge (current) | compatible state/action interface; calibration pending |
| Matched Genesis MPM | calibrated cross-model terrain prediction comparison |
| Gaussian transfer | physically conditioned visual scene update |
| Real-sand calibration | claims tied to measured material and robot data |
