# Limitations and Supported Claims

[Documentation index](README.md)

## Core interpretation boundary

This is a rapid support-planning and visualization prototype. It combines real
Chrono SCM deformation with simplified robot contact and, in traversal mode,
open-loop kinematic trunk motion. Visual quality does not upgrade the fidelity
of the underlying robot dynamics.

## Known limitations

### Traversal trunk is kinematic

The Go1 trunk is fixed, collision-disabled, and moved by direct pose updates.
It cannot respond to contact force, slip, support loss, or foot settlement.
Hazard mode is the exception after its geometric trigger: it releases a rigid
locked-leg proxy with two retained support pads, but still has no articulated
links or contact-force transmission from the visual gait feet.
Difficult-terrain mode deliberately commands trunk roll, pitch, and elevation
from rigid support heights. Its visible tilt is not a dynamic balance response.

### Feet are independent bodies

Traversal feet carry stance mass and deform SCM but are not connected to the
trunk. Their forces do not create trunk acceleration or joint torque.

### No locomotion controller

There is no state estimator, whole-body controller, motor model, torque limit,
balance recovery, or fall detector. `VelocityCommand` is an input to an
open-loop gait generator, not a production velocity controller.

### Static proxy is not an articulated robot

The stationary model is one rigid body with four collision pads. Its body tilt
is physically simulated, but leg compliance and mass distribution are not.

### Spot appearance is incomplete

Spot uses proxy geometry. The PyVista backend currently accepts only Go1.

### Soil is uncalibrated

SCM parameters were selected for visible deformation. Results are not validated
against a specific sand, moisture state, packing density, or laboratory test.

### Cross-model residual response is not calibrated

The companion Genesis bridge has a qualified 5 mm Chrono oracle and an
accepted 5 mm-particle/n128 initial state. Its confirmed `20.433 kPa`,
`14.727 deg`, `0.101895` incumbent reaches `1.864 mm` loaded RMSE, but
residual-footprint RMSE is `13.678 mm` and Genesis is `12.941 mm` too high on
average after removal. This supports a measured cross-model discrepancy, not
a claim of calibrated agreement.

Retained-raw replay `ykep3esa` visualizes that discrepancy with aligned
isometric surface point clouds and signed 2D DEM error. It is not additional
confirmation evidence: four residual projection cells exceeded the frozen
three-cell sparse-bin allowance even though aggregate metrics and p99 map
agreement were stable. The bound was not relaxed; `r2at0vvb` remains the
confirmed run.

A non-learned diagnosis localizes a `9.213 mm` footprint recovery error and a
loaded/residual Pareto trade-off, but it also exposes end-to-end timestep
sensitivity. Halving Genesis timestep changes residual-footprint RMSE by
`1.525 mm` at n128 and `2.325 mm` at n64. The evidence therefore suggests a
constitutive recovery limitation but does not yet isolate model form from
numerical integration/state-preparation effects.

An attempted n128 `0.125 ms` third level could not pass the frozen pre-contact
equilibrium gate with either a 2 or 4 s preparation cap. Reusing the accepted
`0.25 ms` prepared state also failed candidate relaxation at `0.125 ms` before
contact. Consequently there is no third response score or convergence-order
claim.

Controlled traces from the identical accepted state show why: at `0.5 ms`
the fastest 1% is wall/ground localized and persistent movers settle
`-3.135 mm` in median z; at `0.125 ms` the fastest 1% is 99.87%
free-surface localized and persistent movers rise `+2.555 mm`. Fine-step
p50/p95/p99 are `0.291/0.764/0.986 mm/s`. This supports a claim of
timestep-dependent boundary/free-surface preparation dynamics, not uniform
bulk compaction. It does not support relaxing the gate or claiming numerical
convergence.

### Newton mechanics are qualified; material response is not validated

Newton is an alternate MPM backend on a separate branch, not a drop-in
replacement. The companion branch qualifies a fresh 307,461-particle PIC
preparation matrix at `0.5`, `0.25`, and `0.125 ms`. All speed/H0 gates pass,
adjacent DEM RMSE is `0.023/0.031 mm`, and the `0.25 ms` result is unchanged
between tolerances `1e-4` and `1e-5`.

The exact guided cylinder is then loaded for `3.595 s` and removed for a
`0.25 s` residual phase in one continuous solver instance. A circumscribed
128-segment contact mesh and explicit projection/guide settings pass the strict
zero-center-penetration gate throughout the full run. The result is
mechanics-qualified with full external I/O, but it remains an uncalibrated
engineering material at one response timestep. Saved particle arrays omit
solver grid/warm-start history and are not restart-qualified. There is no
validated predictive Newton model or calibration; Genesis state fields and
fitted parameters cannot be attributed to Newton.

### Smoke grid is coarse

The reference traversal uses `35 mm` spacing. This is useful for visualization
but coarse for local pressure and footprint-shape analysis.

### DEM is direct simulation geometry

The DEM is not camera-reconstructed geometry. It is a direct query of flat or
mesh-initialized SCM and masks a known boundary-ring artifact. The displayed
quantity is always `current - initial`, so the rolling course's static relief
cancels and only simulated surface change remains.

### Terrain ownership is piecewise

SCM owns only the central `1.2 m x 1.2 m` patch. The rigid perimeter owns
support outside that box. Kinematic support and visual IK therefore use live
SCM height in bounds and rigid-floor height out of bounds. Artifacts generated
before this clamp can show spawn-leg penetration and are not current numerical
references.

### Dirt is procedural appearance

Granular colors, PBR shading, and shadows improve readability but do not model
individual grains.

### Historical artifacts are heterogeneous

Old videos and trial directories span different friction handling, grid
spacing, mass, and code versions. Filenames alone are not sufficient provenance.

## Supported claims now

The current evidence supports statements such as:

- "Chrono SCM produced persistent surface deformation under the configured
  quadruped support proxy."
- "A nominal 12.5 kg Go1 traversal approximation generated discrete SCM
  footprints with maximum sampled active-node sinkage of 27.61 mm in the shown
  coarse-grid scenario."
- "The same live simulation state was rendered with Matplotlib and PyVista/VTK."
- "Body-frame velocity commands were converted into an open-loop diagonal trot
  visual with 12 solved Go1 joint angles."
- "The planner combines view, path, sinkage, tilt, and uncertainty terms in one
  weighted objective."
- "An offset rigid obstacle triggered a reduced-order release with prescribed
  lateral velocity and one-sided support loss; Chrono then evolved a visible
  `0.332 m` skid and rigid-body fall."
- "The open-loop Go1 visual completed a three-pad rigid course while a fitted
  support plane commanded up to `11.8 deg` of trunk tilt."
- "A guided 5 mm Chrono cylinder episode supplies a fixed-time oracle over
  14,161 valid cells."
- "A ratio-matched 5 mm-particle/n128 Genesis bed passes the documented H0,
  speed, and no-action initialization gates."
- "A controlled two-resolution/two-timestep Genesis matrix exposes material
  timestep sensitivity and does not yet demonstrate numerical convergence."
- "The Newton branch passes its PIC preparation and guided-cylinder mechanics
  gates; response timestep convergence and material calibration remain."

## Claims not supported now

Do not claim:

- stable autonomous walking on sand;
- physically valid Go1 body dynamics during traversal;
- controller robustness or failure probability;
- calibrated prediction of real Go1 sinkage;
- granular particle simulation;
- ROS 2 controller integration;
- calibrated Genesis MPM agreement across loaded, residual, and held-out cases;
- calibrated or validated Newton MPM agreement;
- learned Gaussian deformation;
- an articulated contact-force-caused robot fall;
- controller-stabilized difficult-terrain locomotion;
- sensor-derived DEM reconstruction.

## Hazard/fall labeling

Until articulated dynamics exist, a fall demonstration must state its level:

- **scripted visual fall**: an animation triggered by a chosen condition;
- **reduced-order Chrono fall**: a released rigid trunk with real gravity and
  collision, but no physical legs;
- **articulated robot fall**: connected multibody dynamics and contact.

The current posterity hazard video is the reduced-order level and includes an
on-screen approximation label. Its geometric toe-strike trigger, one-sided pad
selection, prescribed lateral velocity, and one-time angular velocity are not
evidence of articulated stability failure.

## Reproducibility requirements

Any result used externally should record:

- complete YAML values;
- smoke/full-resolution mode;
- CLI command;
- mass scale;
- software versions;
- output metadata;
- deformation metric definition;
- whether motion was stationary, independent-foot traversal, or reduced-order
  hazard mode;
- renderer, while noting that renderer choice does not alter physics.
