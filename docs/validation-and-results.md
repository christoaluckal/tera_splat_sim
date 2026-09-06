# Validation and Results

[Documentation index](README.md)

## Automated tests

Current test suite:

| Test | Purpose |
|---|---|
| zero command symmetric stance | Gait holds nominal four-foot stance |
| forward command diagonal pairs | FR/RL and FL/RR phase behavior |
| yaw changes left/right strides | Body yaw command affects side stride |
| VTK regular grid connectivity | Terrain points and quad indexing |
| soil colors deterministic/darken | Repeatability and compaction appearance |
| hazard offset-track strike | Block catches the intended foot track |
| hazard center/opposite-track miss | Geometric trigger rejects other tracks |
| hazard clearance miss | Raised foot clears a lower obstacle |
| opposite-side support selection | Retained pads switch with hazard side |
| difficult-course overlapping height | Highest rigid pad controls support |
| difficult-course roll and clamp | Offset support creates bounded roll |
| post-pad stance touchdown | Foot center returns to base surface height |
| local-surface swing clearance | Swing Z is measured above current pad height |
| rolling SCM mesh initialization | Hills, valleys, and near-zero boundaries survive Chrono initialization |
| outside-pit support height | Spawn location resolves to rigid-floor elevation |
| flat SCM initialization | Original zero-height constructor remains unchanged |
| maneuver phase sequence | Forward, signed turn, forward, and completion commands |
| maneuver duration | Distance/speed and angle/rate phase timing |
| orbit ring sampling | Multiple theta levels and unique 360-degree phi samples |
| explicit orbit azimuths | Irregular phi lists and angle wrapping |
| camera transform | Rigid OpenGL camera-to-world basis and look direction |
| pinhole intrinsics | VTK vertical field of view to square-pixel focal lengths |
| orbit CLI parsers | Comma-separated angle and XYZ validation |
| metric depth encoding | Float meter depth to uint16 millimeter PNG values |
| heightfield edge skirt | Terrain boundary is closed down to the pit base |
| OpenGL-to-COLMAP pose | Camera look direction becomes positive OpenCV Z |
| COLMAP quaternion round trip | Rotation survives binary-model conversion |
| RGB-D backprojection | Camera-Z center pixel maps to the expected world point |
| inverse-depth encoding | Encoded uint16 values match Frankenstein decoding |

Latest verified result:

```text
Ran 29 tests
OK
```

These are unit tests. There is no automated numerical regression test for a
complete Chrono trial yet.

## Chrono-to-Genesis calibration validation

The active oracle is
`A0_oracle_guided_offset_5mm_gate6mm_v1`: a guided 1.5 kg cylinder on a
5 mm SCM grid, loaded at `3.595 s`, followed by a fixed `0.25 s` residual
observation. Its usable mask contains 14,161 cells.

The companion repository's promoted Genesis bed uses 307,461 particles at
5 mm on n128. It validates with:

| Metric | Value |
| --- | ---: |
| preparation p99 speed | `0.492 mm/s` |
| H0 RMSE | `0.070 mm` |
| H0 maximum error | `0.237 mm` |
| valid cells | `14,161` |

The previous best-known coarse candidate was `E=20 kPa`,
`phi=18.149 deg`, and `nu=0.100004`. The evidence and actions were:

1. n64 result `jg3b5v3s` matched Chrono cylinder sinkage
   (`34.051` versus `34.270 mm`) and scored `8.548 mm`;
2. `vrxqwoe2` added nine valid anchored observations and independently
   confirmed the low-`nu` basin;
3. Genesis particle spacing was reduced to 5 mm when moving to n128, preserving
   the accepted particle-to-grid-cell ratio;
4. geostatic scale, target, physics, loss, and gates remained unchanged;
5. the incumbent and two confirmations were replayed at fixed times.

Results:

| Candidate | n64 objective | n128 objective | n128 loaded RMSE | n128 residual-footprint RMSE |
| --- | ---: | ---: | ---: | ---: |
| 20.000 kPa / 18.149 deg / 0.100004 | **`8.548 mm`** | **`9.626 mm`** | **`2.142 mm`** | **`14.966 mm`** |
| 18.110 kPa / 18.984 deg / 0.103989 | `8.605 mm` | `9.833 mm` | `2.188 mm` | `15.290 mm` |
| 20.186 kPa / 18.485 deg / 0.100693 | `8.643 mm` | `10.041 mm` | `2.316 mm` | `15.449 mm` |

The ordering was stable during resolution promotion, and initialization drift
was negligible. Subsequent compact n128 studies `9on0s14j` and `yab3idti`
improved the same frozen comparison. The current candidate is
`E=20.432828 kPa`, `phi=14.727053 deg`, `nu=0.101894536`; exact replay
`r2at0vvb` confirmed objective `8.704 mm`, loaded RMSE `1.864 mm`,
residual-footprint RMSE `13.678 mm`, and residual signed mean `+12.941 mm`.
The replay passed the companion repository's map-level repeatability test.

The remaining failure is residual response: Genesis is still `12.941 mm` too
high on average inside the footprint after cylinder removal.

### Non-learned diagnosis and numerical matrix

The companion diagnostic decomposed 16 unique valid n128 observations into a
four-point loaded/residual Pareto front and generated aligned radial,
cross-section, recovery, and Genesis `F`/`Jp` summaries. The incumbent's
footprint recovery-error RMSE is `9.213 mm`; the final nonzero-`Jp` count is
8,481 particles versus 1,243 initially.

The controlled end-to-end matrix is:

| particles / grid | timestep | loaded RMSE | residual-footprint RMSE |
| --- | ---: | ---: | ---: |
| 10 mm / n64 | `0.5 ms` | `2.298 mm` | `13.448 mm` |
| 10 mm / n64 | `0.25 ms` | `2.979 mm` | `15.773 mm` |
| 5 mm / n128 | `0.5 ms` | `1.864 mm` | `13.682 mm` |
| 5 mm / n128 | `0.25 ms` | `2.468 mm` | `15.207 mm` |

All four observations are valid and use the same Chrono action and fixed map
times. The timestep effect is material at both resolutions, so numerical
convergence is not demonstrated and a constitutive-only diagnosis is not yet
supported.

The requested third n128 level at `0.125 ms` produced no eligible score.
Prepared-bed runs timed out at 2 and 4 s with final p99 speeds of `0.590` and
`0.621 mm/s`, respectively, while their H0 surface errors still passed.
Reusing the accepted `0.25 ms` prepared state with a `0.125 ms` downstream
runtime also failed candidate relaxation before contact. These are rejected
initialization diagnostics, not response observations.

The subsequent 4 s same-state traces distinguish the motion from uniform bulk
compaction:

| timestep | final p50 / p95 / p99 | fastest 1% localization | persistent median dz |
| ---: | ---: | --- | ---: |
| `0.5 ms` | `0.100 / 0.243 / 0.450 mm/s` | 98.4% wall; 76.7% ground | `-3.135 mm` |
| `0.25 ms` | `0.170 / 0.360 / 0.516 mm/s` | 97.7% wall; 49.9% surface | `-1.968 mm` |
| `0.125 ms` | `0.291 / 0.764 / 0.986 mm/s` | 99.87% surface; 58.8% wall | `+2.555 mm` |

Thus the dominant mode changes with timestep from containment settling to
free-surface uplift/rebound. The fine-step p95 exceeds the frozen gate value,
so the rejection cannot be dismissed as only a one-percent wall tail.
Lightweight CSV/JSON/PNG evidence is tracked in
`tera_splat/diagnostics/pre_settle_timestep_20260903`; large solver states
remain in the companion `outputs/` tree.

### Retained raw and comparison evidence

The companion raw replay `ykep3esa` preserved the confirmed candidate's
candidate-preparation, no-action, initial, loaded, and residual MPM states plus
78 sampled rollout PLYs. Its visualization bundle contains:

- separate loaded and residual figures with Chrono and Genesis fixed-isometric
  surface point clouds beside signed 2D DEM-error maps;
- one combined loaded/residual figure;
- compressed raw comparison arrays on the common 5 mm grid;
- six aligned Chrono-grid surface PCDs, each with 14,161 points;
- initial, loaded, and residual raw Genesis PCDs, each with 307,461 particles.

The point-cloud panels plot surface change from each solver's own initial
state, in the shared `bed` frame, with no surface interpolation. The signed
error is Genesis response minus Chrono response.

This is visualization evidence, not a replacement confirmation. The replay
scored `8.705 mm` and retained p99 map agreement below `0.011 mm`, but four
residual cells exceeded the frozen 1 mm sparse-bin threshold while the
allowance is three. The confirmed run remains `r2at0vvb`; no acceptance bound
was changed after seeing the raw replay.

## RGB-D orbit validation

A real offscreen VTK smoke capture rendered two theta rings and four phi values
per ring at `320x240`:

- 8 RGB PNG, 8 float32 NPY depth, and 8 uint16 PNG depth frames;
- all RGB and depth dimensions matched;
- 8 unique camera-to-world poses;
- phi values `0`, `90`, `180`, and `270` degrees;
- theta values `20` and `45` degrees;
- first-frame valid depth range `1.8434-4.6294 m`;
- background encoded as `NaN` in NPY and `0` in PNG.

The smoke dataset is `/tmp/tera_splat_rgbd_smoke` and is intentionally outside
the repository.

## Direct COLMAP and training validation

The 180-view rolling-terrain dataset could not initialize conventional COLMAP
SfM: only 163 image pairs had verified geometry, no pair had 30 inliers, and
just 86 images were connected. This is expected for the smooth synthetic
terrain texture.

The direct exporter instead used the rendered poses and metric depth. COLMAP's
`model_analyzer` accepted the resulting model with:

| Property | Value |
|---|---:|
| registered images | 180 |
| RGB-D seed points | 223,912 |
| observations | 223,912 |
| mean reprojection error | `0.0 px` |

Frankenstein's scene reader loaded all 180 cameras and all seed points. Across
the first full-resolution depth frame, inverse depth decoded with median
absolute error `0.000006616` and maximum absolute error `0.000014067`.

The active documented `train_nomask.py` path was then run for one iteration at
resolution scale 8 with depth enabled, W&B disabled, and densification
disabled. It initialized 223,912 Gaussians, saved iterations 0 and 1, and
reported `Training complete`.

## PyVista temporal validation

The VTK renderer requires an explicit render after actor and scalar updates.
This was validated by encoding two temporal frames and confirming:

- timestamp changed from `0.00 s` to `1.00 s`;
- robot vertices moved;
- 5,292 pixels changed in the 640x368 temporal smoke artifact.

The final 66-frame video was also compared at frames 0, 30, and 65. Tens of
thousands of pixels changed between nonadjacent frames, confirming the output
is not a repeated first framebuffer.

## Verified reference artifact

File:

```text
quick_support_demo/outputs/videos/go1_velocity_trot_pyvista_dem_12p5kg.mp4
```

Metadata:

| Property | Value |
|---|---:|
| codec | H.264 |
| pixel format | `yuv420p` |
| dimensions | `1280x720` |
| frame rate | 6 FPS |
| frame count | 66 |
| duration | 11 seconds |

Final interior active-node deformation:

| Metric | Value |
|---|---:|
| mean sinkage | `13.62 mm` |
| maximum sinkage | `27.61 mm` |
| active nodes | `115` |

The same values were produced by the prior Matplotlib traversal using the same
simulation parameters, as expected because rendering does not change physics.

## Visual checks performed

Extracted startup, mid-crossing, and final frames were inspected for:

- nonblank VTK output;
- complete Go1 geometry;
- camera coverage from near to far floor;
- moving articulated legs;
- shadows and terrain albedo;
- discrete footprint sequence;
- zero-centered DEM color;
- current foot markers;
- readable, nonoverlapping HUD and scalar bar;
- full right-viewport title.

## Verified rigid hazard artifact

File:

```text
quick_support_demo/outputs/videos/go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4
```

The nominal-mass run detected an `FR` strike at `3.852 s`. After release, the
proxy moved `0.332 m` laterally and Chrono rigid-body dynamics produced a final
trunk tilt of `90.0 deg`. At `3.88 s`, the measured lateral skid was `0.01 m`
with `0.9 deg` tilt; at `4.38 s`, it was `0.22 m` with `49.8 deg` tilt. The
H.264 output is `1280x720`, 8 FPS, 56 frames, and 7 seconds. The rigid center
plate and block have no terrain deformation. Approach, contact, skid, and final
frames were inspected for geometry, continuity, HUD accuracy, visible lateral
translation, and post-release attitude change.

## Historical video artifacts

The output directory includes development artifacts generated under different
settings. Important examples:

| File | Meaning |
|---|---|
| `go1_chrono_dirt_visual.mp4` | Early nominal Matplotlib dirt visualization |
| `go1_chrono_dirt_heavy_4x.mp4` | 50 kg visual stress case |
| `go1_loading_dem_nominal_12p5kg.mp4` | Nominal stationary loading plus DEM |
| `go1_velocity_trot_traverse_independent_feet_12p5kg.mp4` | Matplotlib traversal with independent feet |
| `go1_velocity_trot_pyvista_dem_12p5kg.mp4` | Current VTK reference traversal |
| `go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4` | Rigid offset-block reduced-order skid and fall |
| `go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4` | Rigid uneven-course completion with support-plane tilt |
| `go1_rigid_difficult_terrain_side_pyvista_12p5kg.mp4` | Side-view foot-clearance diagnostic |

Files named `*_irrlicht.mp4` and old BMP frame directories are failed black
frame diagnostics and should not be presented as successful outputs.

## Required future regression coverage

Add automated checks for:

1. SCM height-map shape at smoke and full resolution.
2. Nominal stationary Go1 sinkage range.
3. Mass monotonicity for a controlled static load.
4. Terrain reset equivalence across candidate runs.
5. Boundary-ring masking.
6. Video frame count and nonstatic temporal content.
7. VTK DEM zero color and deformation sign.
8. Traversal end pose and footprint count.
9. End-to-end hazard trigger time and trunk tilt regression.
10. Difficult-course completion and maximum-tilt numerical regression.
11. Rolling-course hills, valleys, and zero-height boundary taper.

## Verified difficult-terrain artifact

File:

```text
quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4
```

The nominal `12.5 kg` run reached the configured far-floor endpoint at
`y=0.95 m` and produced `11.8 deg` maximum commanded trunk tilt. The H.264
output is `1280x720`, 6 FPS, 66 frames, and 11 seconds. Maximum-tilt, alternating
support, course-exit, and final frames were inspected for visible rigid pads,
leg motion, unobstructed text, trunk attitude, and a fully visible completion
pose.

The side-view artifact has matching metadata. Frame inspection confirmed the
front-right foot transitions from `55.3 mm` swing clearance at `4.50 s` to
`0.0 mm` stance clearance at `5.50 s`, and remains at `0.0 mm` in the completed
far-floor stance.

## Superseded straight rolling-terrain artifact

File:

```text
quick_support_demo/outputs/videos/go1_rolling_hills_valleys_scm_deformation_pyvista_dem_12p5kg.mp4
```

This artifact predates the outside-pit support clamp. SCM queries at the rigid
spawn region lowered the kinematic trunk and visual feet, so its deformation
and tilt values are not current and should not be cited. The file is retained
only for visual history.

## Verified forward-turn-forward artifact

File:

```text
quick_support_demo/outputs/videos/go1_rolling_scm_forward_turn_forward_pyvista_dem_12p5kg.mp4
```

The nominal `12.5 kg` sequence completed `0.85 m` forward, a `-90 deg` right
turn at `0.8 rad/s`, and `0.90 m` forward. The reported final pose was
`(0.900, -0.249) m`, yaw `0.0 deg`. Final deformation was mean `29.02 mm`,
maximum `73.88 mm`, across 137 nodes. Maximum commanded trunk tilt was
`9.4 deg`. `ffprobe` reports H.264, `1280x720`, 6 FPS, 66 frames, 11 seconds,
and a `430407` byte file. Frames 0, 18, 30, 42,
54, and 65 were inspected for phase labels, stepping during the turn, final
heading, framing, the bent DEM footprint track, and rigid-floor foot placement
at spawn and exit.

## Newton preparation and response validation boundary

The companion Newton branch reproduces the full frozen bed geometry with
static containment and no Genesis state. Its qualified 307,461-particle PIC
matrix covers `0.5`, `0.25`, and `0.125 ms` plus a tolerance refinement at
`0.25 ms`. Every speed/H0 gate passes; adjacent DEM differences are `0.0232`
and `0.0312 mm` RMSE, and the tolerance results are identical.

The branch also executes the exact 1.5 kg guided cylinder action continuously
from fresh preparation through `3.595 s` loaded and `0.25 s` residual phases.
It emits the same external maps and provenance plus raw arrays, PLYs, traces,
impulses, and penetration diagnostics. The corrected contact discretization
has zero particle centers inside the analytic cylinder throughout the full
trace. Raw all-cell Chrono RMSE is `2.396 mm` loaded and `2.646 mm` residual.
Rebuilding from saved particle arrays still adds about `1.157 mm` of settlement,
so those arrays are not restart-qualified state. This qualifies mechanics and
I/O at one response timestep, not material prediction. Response convergence
must precede fresh Newton calibration; Genesis observations do not satisfy it.

The response matrix is specified before running its remaining levels. It uses
the full continuous path at `0.5`, `0.25`, and `0.125 ms` with fixed smoke
material, PIC transfer, solver tolerance, contact construction, action,
observation times, projection, and mask. Each case must retain accepted
preparation, finite full-support output, and zero analytic center penetration.
On the common valid mask, every adjacent pair of loaded-minus-initial and
residual-minus-initial DEMs must remain within `0.5 mm` RMSE and `1.0 mm`
maximum error; loaded cylinder sinkage must remain within `0.5 mm`. Absolute
Chrono RMSE is reported as uncalibrated fit, not used as a convergence gate.
