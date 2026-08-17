# Planning and Outputs

[Documentation index](README.md)

## Candidate objective

For outcome `o`, candidate configuration `q`, and planning weights `w`, the
implemented objective is:

```text
J = w_view * view_cost
  + w_path * path_cost
  + w_sinkage * maximum_foot_sinkage_m
  + w_tilt * maximum_absolute_body_tilt_rad
  + w_uncertainty * uncertainty
```

The selected candidate is the minimum-cost entry. No threshold, classifier, or
hard feasibility rule is currently applied.

## Candidate semantics

### Sand

- base pose near the pit center;
- lower nominal view and path costs;
- four support contacts can load SCM;
- measured sinkage and tilt contribute to cost.

### Rigid

- base pose on the left rigid margin;
- view and path cost penalties of `0.5` each;
- contact occurs with rigid floor geometry;
- foot-bottom-based sinkage is not equivalent to terrain deformation.

The original intent was for one shared objective to produce robot-conditioned
decisions without hard-coding robot names in the planner.

## SupportOutcome schema

In memory, `SupportOutcome` contains:

| Field | Type | Units |
|---|---|---|
| `robot` | string | none |
| `candidate_pose` | string | none |
| `foot_sinkage_m` | NumPy array, 4 values | m |
| `body_roll_rad` | float | rad |
| `body_pitch_rad` | float | rad |
| `com_height_change_m` | float | m |
| `initial_heightmap_m` | 2D NumPy array | m |
| `loaded_heightmap_m` | 2D NumPy array | m |
| `residual_heightmap_m` | 2D NumPy array | m |
| `runtime_s` | float | s |
| `selected_candidate` | string or null | none |
| `total_cost` | float or null | cost units |

Derived properties:

- maximum foot sinkage;
- mean foot sinkage;
- maximum absolute roll/pitch.

## Directory structure

Each `run_demo.py` invocation writes:

```text
quick_support_demo/outputs/trials/<timestamp>/
|-- go1/
|   |-- sand/
|   |   |-- initial_heightmap_m.npy
|   |   |-- loaded_heightmap_m.npy
|   |   |-- residual_heightmap_m.npy
|   |   `-- outcome.json
|   |-- rigid/
|   |   `-- ...
|   `-- summary.json
`-- spot/
    `-- ...
```

Only requested robots and candidates are created.

## outcome.json

The JSON representation omits large arrays and records their shapes instead.
It includes raw scalar fields plus:

```text
initial_heightmap_shape
loaded_heightmap_shape
residual_heightmap_shape
max_sinkage_m
mean_sinkage_m
max_abs_tilt_rad
```

The corresponding arrays remain in the `.npy` files.

## summary.json

Each robot summary contains:

```text
robot
selected_candidate
costs                    mapping candidate -> scalar cost
outcomes                 mapping candidate -> JSON outcome
```

## Video outputs

3D and preview videos are written under:

```text
quick_support_demo/outputs/videos/
```

The CLI accepts an explicit `--output`. Without one, the 3D renderer generates a
timestamped filename encoding robot, mass scaling, DEM, and traversal state.

## Splat RGB-D datasets

Final-state multi-orbit captures are written to the explicit
`--splat-output` directory. They contain aligned RGB PNG, float32 metric depth,
uint16 millimeter depth, `transforms.json`, and `cameras.json`. See
[Splat RGB-D capture](splat-rgbd-capture.md) for file and coordinate schemas,
direct COLMAP export, and the active Frankenstein training command.

## DEM data interpretation

The video DEM is computed live and is not currently persisted as a per-frame
episode. Stationary trials persist only initial, loaded, and residual arrays.
Both flat and rolling SCM videos display `current - initial` elevation change.
The initial map for rolling terrain already contains the hills and valleys, so
their static relief cancels and the DEM isolates contact deformation.

For future MPM or Gaussian replay, a canonical episode should additionally
record:

- every sampled height map and timestamp;
- body and foot transforms;
- contact/stance labels;
- complete configuration snapshot;
- renderer-independent camera calibration;
- software versions and source revision.

See [roadmap and extensions](roadmap-and-extensions.md).
