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

Latest verified result:

```text
Ran 18 tests
OK
```

These are unit tests. There is no automated numerical regression test for a
complete Chrono trial yet.

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
