# Robots and Locomotion

[Documentation index](README.md)

## Go1 asset sources

The repository includes a Unitree Go1 URDF and source visual meshes under
`quick_support_demo/assets/go1`. Simplified STL parts are used to keep frame
generation practical. Asset provenance and licensing are recorded in:

- `quick_support_demo/assets/go1/SOURCE.md`;
- `quick_support_demo/assets/go1/LICENSE`.

## Go1 visual assembly

`robot_assets/go1.py` parses the URDF and computes a transform tree from the
`trunk` root. It supports:

- fixed-joint origins;
- revolute joint axis rotations;
- per-visual origin and scale;
- link-specific simplified mesh selection;
- colors by link category;
- generated spherical feet.

Source meshes are merged, cleaned, converted to convex hulls, and decimated to
target face counts. The standing configuration is cached as compressed NumPy
arrays and OBJ:

```text
quick_support_demo/assets/go1/render_cache/go1_standing.npz
quick_support_demo/assets/go1/render_cache/go1_standing.obj
```

The stationary renderer uses the standing mesh. Traversal calls
`load_go1_articulated_visual` for every captured gait state so all 12 leg joints
are visibly animated.

## Nominal joint convention

The standing visual uses, for every leg:

- hip: `0.0 rad`;
- thigh: `0.9 rad`;
- calf: `-1.8 rad`.

The motion module computes new values from foot targets rather than blending
these nominal values.

## Velocity command

`VelocityCommand` contains:

```text
vx_mps   forward velocity in body frame
vy_mps   lateral velocity in body frame
wz_radps yaw rate about body z
```

The video loop integrates these commands into trunk world position and yaw.
With initial yaw `pi/2`, positive body-frame `vx` carries the robot from the
near floor in positive world `y` across the pit.

## Trot timing

The gait uses diagonal phase pairs:

```text
FR + RL: phase offset 0.0
FL + RR: phase offset 0.5
```

Default parameters:

- frequency: `1.6 Hz`;
- duty factor: `0.58`;
- step height: `0.055 m`.

During stance, a target foot moves backward relative to the body over the duty
interval. During swing, a cosine blend moves it forward and a sine arc raises
it by the requested step height.

Velocity and yaw command affect stride as:

```text
foot_velocity_x = vx - wz * nominal_y
foot_velocity_y = vy + wz * nominal_x
stride = foot_velocity * duty_factor / frequency
```

## Inverse kinematics

Each leg is modeled with:

- front/rear hip x offset `0.1881 m`;
- left/right hip y offset `0.04675 m`;
- lateral hip link offset `0.08 m`;
- thigh and calf lengths `0.213 m` each.

The solver first resolves hip abduction/adduction in the lateral plane, then
solves the two-link sagittal chain for thigh and calf. It rejects unreachable
targets outside numerical tolerance. Forward kinematics is implemented for
validation.

## Contact-adjusted visual pose

Desired stance feet may settle vertically in SCM. Before rendering, the actual
Chrono stance-foot position is transformed back into the trunk frame and passed
through IK again. Swing feet retain desired kinematic targets. This makes the
visual leg approximately meet the settled contact body.

## Current physical model

Stationary mode:

- one dynamic rigid trunk body;
- four collision pads rigidly attached to it;
- no moving leg joints.

Traversal mode:

- one fixed/kinematic trunk body;
- four independent contact bodies;
- visual legs only;
- no links, constraints, motors, or force transfer between trunk and feet.

## Articulated Go1 upgrade specification

Replacing the kinematic robot means changing the physical model, not merely
changing the rendered mesh. The target model has a floating dynamic trunk,
three physical links per leg, 12 actuated revolute joints, and foot contact
transmitted through the connected mechanism. Trunk motion and attitude must
then result from gravity, inertia, joint actuation, and terrain contact rather
than direct `SetPos` and `SetRot` calls.

### Available URDF data

The included `quick_support_demo/assets/go1/urdf/go1.urdf` contains 46 links
and 45 joints. Twelve joints are actuated leg revolutes; the remaining joints
attach the base placeholder, feet, rotors, IMU, cameras, and other sensors as
fixed children. It provides:

- joint origins and axes;
- lower and upper angle limits;
- velocity and effort limits;
- link masses and centers of mass;
- full inertia tensors, including products of inertia;
- primitive collision geometry;
- visual mesh references.

The installed PyChrono 8 Python bindings do not include `pychrono.parsers`, so
this URDF cannot be handed to a built-in Python URDF importer. A local importer
should extend the XML origin, axis, and transform handling already implemented
in `robot_assets/go1.py`.

The URDF inertial entries sum to approximately `13.100528 kg`, while
`configs/go1.yaml` currently declares `12.5 kg`. The articulated implementation
must choose one authoritative mass model. It must not assign the configured
trunk mass in addition to all URDF link masses. Payload mass and its center of
mass should be applied explicitly to the trunk.

### Body consolidation

Creating every fixed URDF child as a separate Chrono body would add unnecessary
weld constraints. Fixed children should instead be merged into their nearest
moving ancestor:

```text
trunk + fixed sensors and hip rotors
4 hip bodies + fixed thigh rotors
4 thigh bodies + fixed calf rotors
4 calf bodies + fixed feet
```

This produces 13 dynamic bodies: one trunk and three bodies per leg. When a
fixed child is merged, its center of mass and inertia must be rotated into the
parent frame and combined with the parallel-axis theorem.

Each physical link should use `ChBodyAuxRef`. Its reference frame represents
the URDF link frame, while `FrameCOMToRef` represents the URDF inertial origin.
This preserves correct joint placement when a link frame does not coincide
with its center of mass.

Visual meshes should remain non-contact geometry. Initial collision geometry
should use the URDF boxes, cylinders, and foot spheres. Robot self-collision
should initially be disabled while foot-to-terrain and trunk-to-obstacle
collision remain enabled.

### Revolute joints and motors

Each actuated URDF joint should be represented by a Chrono rotational motor
with a revolute spindle constraint. The intended final actuator is
`ChLinkMotorRotationTorque`; ideal angle motors are useful only for fixed-base
kinematic validation.

Chrono rotational motors use their local Z axis as the spindle. The motor frame
must therefore rotate local Z onto the URDF axis. Go1 hip axes are URDF X;
thigh and calf axes are URDF Y. Motor angle sign and zero offset must be checked
against the existing visual forward kinematics before enabling gravity.

The URDF limits include:

| Joint group | Effort limit | Velocity limit |
|---|---:|---:|
| hip and thigh | `23.7 Nm` | `30.1 rad/s` |
| calf | `35.55 Nm` | `20.06 rad/s` |

Torque commands must be saturated at these limits. Position limits should
initially use command clamping plus bounded soft-stop torques because the
torque motor does not directly enforce the URDF lower and upper stops.

### Actuation modes

The articulated model should expose three explicit modes:

1. `angle`: ideal angle motors for URDF transform and renderer validation;
2. `pd`: torque motors with local joint-space PD control;
3. `effort`: externally supplied torques for whole-body or MPC controllers.

A bring-up PD controller can use:

```text
tau = clamp(Kp * (q_des - q) + Kd * (qd_des - qd), torque_limit)
```

The current `TrotGait` and analytical IK may supply initial desired joint
trajectories. They do not by themselves stabilize a floating robot and must
not be treated as a balance controller.

### Contact and SCM consequences

The articulated model replaces `independent_feet.py`; stance mass is no longer
redistributed manually. Each link retains its URDF mass, and contact forces at
the physical feet propagate through the joints to the trunk.

The URDF foot collision radius is `0.02 m`, while the current SCM proxy uses
`0.035 m`. That difference changes projected contact area and soil pressure by
roughly a factor of three. Foot geometry must therefore be selected and
calibrated explicitly before comparing articulated sinkage with existing proxy
videos.

The existing `0.0005 s` physics timestep supports a 2 kHz Chrono update. A
joint controller can run at 500-1000 Hz while state publication and planning
run more slowly. The simulation step order should be:

```text
read base, joint, and contact state
compute and limit 12 actuator torques
SCM Synchronize
Chrono DoStepDynamics
SCM Advance
publish updated state
```

### Terrain attitude after replacement

Articulated mode must remove direct support-plane trunk position and rotation
updates. The fitted support plane may remain as a desired body-attitude input
to a controller, but it must not overwrite physical state. Actual roll and
pitch must result from terrain contact, joint torques, gravity, mass
distribution, and controller response.

### Commissioning sequence

The model should be introduced behind a separate mode such as
`--robot-dynamics articulated`, preserving current proxy artifacts. Bring-up
should proceed through these gates:

1. Parse the URDF and validate the consolidated mass and inertia.
2. Compare fixed-base physical forward kinematics with the existing visual FK.
3. Replay standing and gait poses with ideal angle motors and no contact.
4. Establish a floating-base PD stand on rigid flat ground.
5. Verify that summed ground reaction approximately equals robot weight.
6. Walk on rigid flat ground without direct trunk pose updates.
7. Add rigid uneven terrain and observe physically generated attitude.
8. Introduce SCM only after foot pressure and contact stability are validated.
9. Expose base, IMU, joint, effort, and contact state through ROS 2.
10. Replace local PD references with an external whole-body controller.

The first controller-ready acceptance target is a floating articulated Go1
that stands on rigid ground for 10 seconds with bounded torques, no direct
trunk pose writes, finite constraint errors, and physically generated body
attitude. Walking and SCM deformation come after that gate.

## Fall behavior

In normal SCM traversal, the loop writes trunk position and rotation every time
step while the body is fixed and collision-disabled. Foot settlement does not
change trunk attitude, so support-polygon loss, one-sided collapse, and slip
cannot cause the trunk to roll or pitch.

Hazard mode implements the second of these three fidelity levels:

1. **Visual scripted fall**: fast, but must be labeled animation.
2. **Reduced-order Chrono fall (implemented)**: detect a geometric foot/block
   strike, stop gait, retain locked collision pads opposite the failed side,
   and release the low-friction rigid proxy with documented lateral and angular
   velocity. The pads prevent the unsupported vertical drop seen in the first
   version and make the pre-fall skid visible.
3. **Articulated physical fall**: import a connected multibody Go1, add joint
   actuation, and let contact forces destabilize the system.

The implemented level 2 video is a rigid obstacle showcase. Level 3 remains
required for claims about quadruped stability or controller behavior.

## Difficult-terrain completion

Difficult mode does not release or fall. The gait keeps integrating the
`VelocityCommand` until the trunk reaches `y=0.95 m` on the far floor. Its
elevation, roll, and pitch follow a smoothed plane fitted to the four commanded
foot support heights. The `0.10 m` reference swing height clears the tallest
`0.085 m` pad.

This produces readable leg stepping and trunk tilt while preserving completion,
but it is not balance control. Visual IK uses terrain-adjusted commanded gait
targets rather than independent-foot collision transients. Stance targets lie
exactly at the local rigid surface, while swing targets add the gait clearance
above that surface.

The same gait adaptation is used by `--rolling-terrain`. In that mode the four
support samples come from the live deforming SCM surface rather than pad
lookups, so body elevation can be negative in a valley or contact depression.
The corrected maneuver reaches `9.4 deg` maximum commanded resultant trunk
tilt. Out-of-pit samples use rigid-floor height rather than SCM height.
This remains prescribed kinematic terrain following; it is not evidence of
dynamic stability or controller performance.

## Forward-turn-forward command sequence

`ForwardTurnForward` schedules body-frame velocity commands by elapsed time.
The two translation phases command positive `vx`; the turn phase commands zero
translation and signed `wz`, so the existing yaw-aware trot produces different
left/right stride targets while the trunk turns in place.

The default sequence is `0.85 m` forward, `-90 deg` at `0.8 rad/s`, then
`0.90 m` forward. Starting at yaw `90 deg`, the right turn finishes at yaw
`0 deg`, changing travel from world `+y` to world `+x`. The verified final pose
is `(0.900, -0.249) m`. This scheduler has no localization, path-error feedback,
foothold planning, or obstacle response.

## ROS 2 relationship

No ROS 2 controller is currently used. A future bridge should map a command
such as `geometry_msgs/Twist` to `VelocityCommand`, but that alone is not a
controller. A practical stack also needs:

- state estimation;
- gait scheduling;
- foothold planning;
- body and swing-foot trajectory generation;
- whole-body control or inverse dynamics;
- joint position/velocity/torque interfaces;
- contact and fall detection.
