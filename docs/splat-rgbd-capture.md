# Splat RGB-D Capture

[Documentation index](README.md)

## Purpose

The PyVista renderer can freeze the final state of a Chrono run and capture it
from concentric camera rings. Each ring has a configurable elevation `theta`
and samples one or more azimuths `phi`. This produces calibrated RGB-D views
for transforms-based NeRF and Gaussian-splatting pipelines.

The capture contains rendered ground-truth camera poses. Pipelines that accept
`transforms.json` and optional depth can consume it directly. The included
RGB-D-to-COLMAP exporter converts those poses and initializes a colored sparse
point cloud without running feature matching or structure from motion.

## Example

This command runs the forward-turn-forward episode, writes its normal video,
then captures the final deformed scene using four elevation rings and 36
azimuths per ring:

```bash
MPLCONFIGDIR=/tmp/matplotlib-chronos XDG_CACHE_HOME=/tmp/chrono-cache \
conda run -n chrono_splat python -m quick_support_demo.overlays.make_chrono_3d_video \
  --robot go1 --renderer pyvista --traverse --rolling-terrain \
  --forward-turn-forward --vx 0.25 \
  --first-forward-distance 0.85 --turn-angle-deg -90 --turn-rate 0.8 \
  --second-forward-distance 0.90 \
  --gait-frequency 1.6 --step-height 0.10 \
  --duration 11 --fps 6 --dem-panel --dem-max-mm 140 --smoke \
  --width 1280 --height 720 \
  --splat-output quick_support_demo/outputs/splat_datasets/go1_ftf_final \
  --splat-hide-robot \
  --orbit-theta-deg 15,30,45,60 --orbit-phi-count 36 \
  --orbit-radius 3.2 --orbit-target 0,0,0.15 \
  --orbit-view-angle-deg 45 \
  --output quick_support_demo/outputs/videos/go1_ftf_capture_source.mp4
```

The default orbit produces `4 * 36 = 144` RGB-D views. The output directory
must not already exist, which prevents stale frames from an older sampling
configuration from remaining in a dataset.

Use explicit irregular azimuths when needed:

```text
--orbit-theta-deg 10,25,40,55 --orbit-phi-deg 0,20,55,90,140,200,270,320
```

When `--orbit-phi-deg` is present, it overrides `--orbit-phi-count` and
`--orbit-phi-offset-deg`.

## COLMAP and Gaussian Splatting export

Synthetic terrain can have too little local texture for COLMAP's feature
matcher to find a reliable initial image pair. Do not run the standard
`convert.py` reconstruction on this renderer output. Export the known camera
poses and RGB-D geometry directly instead:

```bash
conda run -n frankenstein python -m \
  quick_support_demo.overlays.export_colmap_from_rgbd \
  -s quick_support_demo/outputs/splat_datasets/rolling_terrain_no_robot_180 \
  --resize
```

Run this command from the `tera_splat_sim` repository root. It creates
`images`, `images_2`, `images_4`, `images_8`, `invdepth`, and `sparse/0`.
The COLMAP model uses the renderer's exact intrinsics and poses. Its initial
`points3D.ply` is formed by voxel-downsampling back-projected RGB-D pixels.

The active Frankenstein trainer can then use the dataset without COLMAP
reconstruction. Run it from the `gaussian-splatting` repository:

```bash
cd ~/Workspace/splatting/frankenstein/gaussian-splatting
conda run -n frankenstein python train_nomask.py \
  -s ~/Workspace/splatting/physical/Chronos/tera_splat_sim/quick_support_demo/outputs/splat_datasets/rolling_terrain_no_robot_180 \
  -m output/rolling_terrain_no_robot_180 \
  -d invdepth -r 1 --resolution_scales 1 \
  --disable_viewer --disable_wandb
```

Use `-d invdepth`, not `-d depth_png`. The former stores calibrated inverse
camera-Z depth in the convention expected by that loader; `depth_png` stores
metric millimeters for general RGB-D consumers. `-r 1` avoids applying a
second resize on top of `--resolution_scales`; use `--resolution_scales 8`
only for a fast, low-resolution plumbing check.

The minimal command used to validate this dataset was:

```bash
conda run -n frankenstein python train_nomask.py \
  -s ~/Workspace/splatting/physical/Chronos/tera_splat_sim/quick_support_demo/outputs/splat_datasets/rolling_terrain_no_robot_180 \
  -m /tmp/chrono_rgbd_train_nomask_smoke \
  -d invdepth -r 1 --resolution_scales 8 \
  --iterations 1 --test_iterations 1 --save_iterations 0 1 \
  --disable_viewer --disable_wandb --no-densify
```

## Dataset layout

```text
go1_ftf_final/
|-- images/
|   |-- frame_00000.png           uint8 RGB
|   `-- ...
|-- depth/
|   |-- frame_00000.npy           float32 camera-Z depth in meters
|   `-- ...
|-- depth_png/
|   |-- frame_00000.png           uint16 camera-Z depth in millimeters
|   `-- ...
|-- invdepth/                      created by the COLMAP exporter
|   |-- frame_00000.png           uint16 normalized inverse camera-Z depth
|   `-- ...
|-- sparse/0/                      created by the COLMAP exporter
|   |-- cameras.bin               exact pinhole intrinsics
|   |-- images.bin                exact COLMAP/OpenCV world-to-camera poses
|   |-- points3D.bin              RGB-D seed points and tracks
|   |-- points3D.ply              Gaussian Splatting initialization
|   `-- depth_params.json         inverse-depth scale and offset
|-- transforms.json               intrinsics and OpenGL camera-to-world poses
`-- cameras.json                  explicit conventions and inverse poses
```

Every RGB, NPY depth, and PNG depth file has the same dimensions and frame
index. The images contain no HUD or DEM panel. A video may still use
`--dem-panel`; orbit capture always creates a separate full-frame scene.
Cast shadows are disabled in this scene to avoid view-dependent self-shadow
artifacts becoming false geometry or appearance signal during reconstruction.
The heightfield also includes four vertical edge skirts down to the pit base so
RGB and depth remain closed where a rolling boundary falls below the rigid
perimeter.

## Camera convention

The simulation world is right-handed and Z-up.

- `phi` is azimuth from world `+X` toward world `+Y`.
- `theta` is elevation above the world XY plane.
- `phi` samples cover `[0, 360)`; the endpoint is excluded to avoid a duplicate
  camera at the seam.
- `transform_matrix` is OpenGL/NeRF camera-to-world.
- Camera `+X` is image-right, `+Y` is image-up, and camera `-Z` is the viewing
  direction.
- Intrinsics are square-pixel pinhole values computed from VTK's vertical
  field of view. Distortion coefficients are zero.

`cameras.json` also stores `world_to_camera_opengl` for each frame so consumers
do not need to infer matrix direction.

## Depth convention

Depth is positive distance along the camera viewing axis, not Euclidean range
along each pixel ray.

- `depth/*.npy`: `float32` meters; background is `NaN`.
- `depth_png/*.png`: `uint16` millimeters; background is `0`.
- `invdepth/*.png`: normalized `uint16` inverse depth; background is `0`.
- Valid PNG values saturate at `65535 mm`.
- `transforms.json` records `depth_unit_scale_factor: 0.001` and each frame's
  `depth_file_path`.
- `cameras.json` records both depth paths and the full encoding contract.

Use the NPY data when preserving sub-millimeter precision or `NaN` background
is important. Use the PNG data for training loaders that expect integer depth
images. The `invdepth` images and their shared scale in
`sparse/0/depth_params.json` are specifically for the upstream Gaussian
Splatting depth loader.

## Capture controls

| Flag | Default | Meaning |
|---|---:|---|
| `--splat-output` | none | New output directory; enables final-state RGB-D capture |
| `--splat-hide-robot` | off | Exclude robot geometry from orbit RGB and depth |
| `--orbit-theta-deg` | `15,30,45,60` | Elevation rings in degrees |
| `--orbit-phi-count` | `36` | Uniform azimuth samples per ring |
| `--orbit-phi-deg` | none | Explicit azimuth list, overriding uniform sampling |
| `--orbit-phi-offset-deg` | `0` | Rotation applied to uniform azimuth samples |
| `--orbit-radius` | `3.2` | Camera-to-target radius in meters |
| `--orbit-target` | `0,0,0.15` | Common world-space look-at point in meters |
| `--orbit-view-angle-deg` | `45` | Vertical perspective field of view |
| `--width`, `--height` | `1280`, `720` | RGB and depth dimensions |

The capture represents the state at `--duration`. To capture another instant,
change the run duration. The current CLI captures one frozen simulation state;
it does not yet capture all orbit views at every simulation frame.

## Training caveats

The renderer uses a fixed procedural background and deterministic scene
lighting. Depth includes the robot, terrain, perimeter slabs, and target wall
visible to VTK. The terrain albedo is procedural rather than photorealistic.

For reconstruction quality, use enough overlap between adjacent views. The
default 10-degree azimuth spacing is conservative. Low rings preserve side
detail; high rings expose the terrain and deformation. Avoid `theta` near
`+/-90` because the Z-up camera basis becomes singular.
