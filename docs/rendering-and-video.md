# Rendering and Video

[Documentation index](README.md)

## Rendering architecture

Chrono advances the physical state and SCM deformation. Rendering occurs only
at requested video frame times. Both 3D backends consume the same sampled
height map, body transform, gait state, and metrics.

`imageio` writes RGB frames through FFmpeg using H.264 (`libx264`). Output sizes
should be divisible by 16 for broad codec compatibility. The default
`1280x720` is valid.

## Matplotlib backend

Select with:

```text
--renderer matplotlib
```

This is the default and most portable path. It uses the noninteractive Agg
backend and draws:

- a 3D surface from sampled SCM elevations;
- deterministic granular color variation;
- baked slope-dependent illumination;
- rigid floor boxes and target;
- Go1 triangle geometry or Spot proxy geometry;
- text metrics;
- optional 2D DEM-difference panel.

Strengths:

- deterministic headless behavior;
- no OpenGL dependency;
- supports Go1 and Spot stationary visuals;
- easy scientific annotation.

Limitations:

- slow Python-level 3D rendering;
- no physically based materials;
- limited shadow and camera quality;
- terrain texture remains procedural rather than photographic.

## PyVista/VTK backend

Select with:

```text
--renderer pyvista
```

The backend is implemented in `overlays/pyvista_renderer.py` and currently
supports the Go1 visual asset. It creates one persistent scene and updates it in
place instead of rebuilding actors for each frame.

Scene features:

- regular quad terrain mesh from the SCM height map;
- deformation-dependent brown granular point colors;
- PBR material settings;
- smooth terrain and robot shading;
- separate key and fill lights;
- cast shadows;
- FXAA;
- fixed camera covering the entire traversal;
- dynamic HUD;
- optional second VTK viewport for DEM difference.

The robot mesh uses per-cell RGB colors and updates its vertices from the
current articulated visual pose. An explicit VTK render is issued before every
framebuffer readback so actor updates are encoded rather than reusing the first
frame.

## VTK DEM panel

The VTK DEM is a flat quad mesh colored by signed elevation difference. It uses
a reversed red-white-blue map and symmetric limits:

```text
[-dem_max_mm, +dem_max_mm]
```

This guarantees:

- negative/subsided regions are red;
- unchanged terrain is white;
- uplift is blue.

The boundary ring is `NaN` and displayed neutral gray. Current foot positions
are overlaid as black points. Text reports mean and maximum subsidence and
maximum uplift.

The Matplotlib DEM uses an asymmetric normalization with zero explicitly
centered and a smaller positive uplift range. Colors between backends are
qualitatively consistent but their numeric color scales are not identical.

## Dirt appearance

The visible granular effect is a deterministic multiscale sinusoidal albedo,
not explicit particles. Deformed nodes darken toward a compacted-soil color.
The VTK backend then applies scene lighting and PBR roughness.

This is suitable for communicating footprints and surface relief. It should not
be described as grain-resolved rendering.

## Rigid hazard rendering

Hazard mode draws a neutral gray rigid center plate and a contrasting fixed
block at the configured offset. Because there is no SCM, the surface remains
flat and the HUD reports `Rigid course deformation: none`. The renderer also
shows the detected strike leg, lateral skid distance, trunk tilt, and
`Reduced-order skid + rigid-body fall; no controller` so the approximation
remains visible in the exported artifact. A DEM panel is not meaningful for
this mode.

## Difficult-course rendering

The rigid center plate is gray. Three raised pads use orange, blue, and yellow
so alternating support sides remain legible from the fixed camera. The HUD
reports current trunk tilt, zero deformation, and `Kinematic terrain following;
no balance controller`. The robot remains visible after reaching the far floor;
there is no final footprint fade or DEM panel.

`--side-view` moves the camera to the positive world-x side, looking across the
course along the direction of travel. In difficult mode its HUD adds the
front-right foot-bottom clearance and current stance/swing state. This view is
intended to reveal pad clearance, touchdown, and any floating-foot error.

## Rolling-terrain DEM rendering

With `--rolling-terrain --dem-panel`, the left VTK viewport remains the normal
perspective traversal. The right viewport is a top-down deformation DEM, not a
second camera. It reports `current surface - initial surface`, mean and maximum
subsidence, and maximum uplift in millimeters.
Black point markers show the terrain-adjusted visual foot x/y locations.

The scene surface and DEM use the same height map. The scene uses deterministic
earth-tone elevation coloring with fine procedural grain; the DEM uses the
zero-centered `coolwarm_r` map used by flat SCM runs. `--dem-max-mm 90` is a
useful default for the corrected floor-clamped model; the checked maneuver
artifact was rendered at `140 mm` and therefore uses a lighter color range.

## Graphics requirements

The installed versions are:

- PyVista `0.48.4`;
- VTK `9.6.2`.

On this workstation, offscreen VTK succeeds through EGL only when the process
has graphics-device access. A restricted sandbox can import VTK but fail or
crash during EGL initialization.

Headless alternatives:

1. use an EGL-enabled VTK wheel/build with device access;
2. use an OSMesa-enabled VTK build;
3. run an X-capable VTK build under Xvfb;
4. use Matplotlib when none of those are available.

`MPLCONFIGDIR` and `XDG_CACHE_HOME` should point to writable temporary
directories to avoid Matplotlib/font cache warnings.

## Irrlicht status

The repository includes an interactive Chrono Irrlicht scene. Interactive
initialization can work with a display, but prior screenshot attempts under
Xvfb/Mesa produced black frames. Irrlicht is not the recommended recorded-video
path in the current environment.

## Generated PyVista reference video

```text
quick_support_demo/outputs/videos/go1_velocity_trot_pyvista_dem_12p5kg.mp4
```

Verified properties:

- H.264 High profile;
- YUV 4:2:0 pixel format;
- `1280x720`;
- 6 FPS;
- 66 frames;
- 11 seconds;
- synchronized live VTK scene and DEM.

## Generated rigid hazard video

```text
quick_support_demo/outputs/videos/go1_rigid_offset_hazard_slip_pyvista_12p5kg.mp4
```

Verified properties:

- H.264 with YUV 4:2:0 pixel format;
- `1280x720` at 8 FPS;
- 56 frames and 7 seconds;
- front-right strike at `3.852 s`;
- final lateral skid `0.332 m`;
- final trunk tilt `90.0 deg`;
- zero terrain deformation.

## Generated difficult-terrain video

```text
quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_tilt_pyvista_12p5kg.mp4
```

Verified properties:

- H.264 with YUV 4:2:0 pixel format;
- `1280x720` at 6 FPS;
- 66 frames and 11 seconds;
- maximum commanded trunk tilt `11.8 deg`;
- complete crossing to `y=0.95 m`;
- zero terrain deformation.

Side-view diagnostic:

```text
quick_support_demo/outputs/videos/go1_rigid_difficult_terrain_side_pyvista_12p5kg.mp4
```

It uses the same simulation settings and video metadata. Verified checkpoints:

- `t=4.50 s`: front-right swing clearance `55.3 mm`;
- `t=5.50 s`: front-right stance clearance `0.0 mm` after leaving the first pad;
- final pose: front-right stance clearance `0.0 mm` on the far floor.

## Historical straight rolling-terrain DEM video

```text
quick_support_demo/outputs/videos/go1_rolling_hills_valleys_scm_deformation_pyvista_dem_12p5kg.mp4
```

This file predates outside-pit floor clamping and is superseded for numerical
claims. Its spawn support queried SCM outside the patch. Retained properties:

- H.264 at `1280x720`, 6 FPS, 66 frames, and 11 seconds;
- complete crossing to `y=0.95 m`;
- initial terrain range `-64.2 mm` to `+76.9 mm`;
- perspective traversal on the left and deformation DEM on the right.

## Generated forward-turn-forward video

```text
quick_support_demo/outputs/videos/go1_rolling_scm_forward_turn_forward_pyvista_dem_12p5kg.mp4
```

Verified properties:

- H.264 at `1280x720`, 6 FPS, 66 frames, and 11 seconds;
- HUD phases `warmup`, `forward 1`, `turn right`, `forward 2`, and `complete`;
- final pose `(0.900, -0.249) m`, yaw `0.0 deg`;
- maximum commanded trunk tilt `9.4 deg`;
- final mean subsidence `29.02 mm` across 137 nodes;
- final maximum subsidence `73.88 mm`;
- feet remain on the rigid perimeter at spawn and exit;
- bent footprint track visible in the synchronized DEM.
