# Quick Visual Demo: Robot-Conditioned Planning over a Deformable Sand Pit

> **Document status:** Original requirements and storyboard. For the canonical
> current implementation documentation, start at [`docs/README.md`](../README.md).

## 1. Demo objective

Build a short simulated demonstration in which the same environment produces different support decisions for Go1 and Spot.

The demo should communicate one idea clearly:

> Traversability is not only a property of a terrain patch. It depends on the robot's mass, foot geometry, payload, and intended contact pose.

The minimum viable demo does **not** require:

- full quadruped walking on sand;
- a trained Gaussian reconstruction;
- online Genesis MPM rollouts;
- calibrated real-sand parameters;
- ROS 2 integration;
- an autonomous locomotion controller.

It requires a convincing scene, visible terrain deformation, two candidate poses, and a robot-dependent planning result.

---

## 2. Recommended implementation stack

### Minimum visual demo

| Component | Tool | Responsibility |
|---|---|---|
| World and robot dynamics | Project Chrono | Rigid environment, robot pose, gravity, contact |
| Deformable pit | Chrono SCM terrain | Fast visible sinkage and persistent rutting |
| Robot assets | Go1 and Spot URDFs | Appearance, mass, dimensions, nominal stance |
| Runtime display | Chrono VSG visualizer | Main 3D demonstration window |
| Experiment control | PyChrono or a small C++ executable | Reset, change robot, select pose, execute trial |
| Result overlay | Python/OpenCV or lightweight UI | Candidate markers, costs, sinkage and tilt labels |

### Physical-GS extension

| Component | Tool | Responsibility |
|---|---|---|
| Simulated observations | Chrono Sensor | RGB, depth, segmentation and camera poses |
| Persistent scene map | 3DGS pipeline | Metric geometry, appearance, semantics |
| Counterfactual predictor | Genesis MPM | Predict deformation for candidate robot contacts |
| GS state transition | MPM-to-GS transfer | Produce a predicted post-contact Gaussian scene |

Start with the minimum visual demo. Treat Chrono SCM as an oracle support model. Add Genesis MPM and the Gaussian map only after the scene is stable.

---

## 3. Exact scene

### World dimensions

- Overall floor: `3.0 m x 3.0 m`
- Central sand opening: `1.2 m x 1.2 m`
- Visual pit depth: `0.15 m`
- Rigid walking floor elevation: `z = 0`
- SCM terrain surface elevation: `z = 0`
- Inspection target: approximately `1.0-1.5 m` beyond the far edge of the pit

### Floor construction

Do not put one rigid plane directly underneath the entire SCM contact surface at `z = 0`. Construct the visible floor from four rigid boxes surrounding a central opening:

```text
                         INSPECTION TARGET
                               [ TAG ]

    +--------------------------------------------------+
    |                    rigid floor                   |
    |                                                  |
    |   rigid candidate                                |
    |         O                                        |
    |                                                  |
    |          +--------------------------+            |
    |          |                          |            |
    |          |      SCM SAND PIT        |            |
    |          |       1.2 x 1.2 m        |            |
    |          |                          |            |
    |          +--------------------------+            |
    |                 O sand candidate                 |
    |                                                  |
    |                     START                        |
    +--------------------------------------------------+
```

The four floor boxes form the left, right, near, and far margins. Add four thin wall meshes around the pit for appearance if desired, but keep them out of foot contact regions.

### Visual environment

Use a simple research-lab aesthetic:

- neutral gray rigid floor;
- tan granular terrain texture;
- dark pit walls or frame;
- one high-contrast inspection board or AprilTag target;
- two colored candidate-pose markers;
- a low wall, workbench, or background panels to provide scale;
- soft directional lighting plus ambient fill;
- no clutter near the robot or pit.

The inspection target gives the robot a reason to prefer the closer sand pose. Without a task target, choosing where to stand looks arbitrary.

---

## 4. Robot models

### Required assets

- Unitree Go1 URDF and meshes
- Boston Dynamics Spot URDF and meshes
- Nominal standing joint configuration for each robot
- Simplified foot collision geometry for each robot
- Correct total mass or an explicit mass override

### First robot model: locked stance

For the first version:

1. Import the URDF.
2. Put the robot in a nominal standing pose.
3. Lock the joints or hold them with stiff position motors.
4. Leave the base floating.
5. Replace detailed foot collision meshes with cylinders or spheres.
6. Initialize the feet `2-5 mm` above the terrain.
7. Release the robot under gravity.
8. Simulate until vertical speed and angular velocity are below thresholds.

This allows:

- redistribution of contact load between the four feet;
- differential settlement;
- body roll and pitch;
- visible terrain deformation.

It does not claim to simulate walking or the production locomotion controller.

### Debug model

Before importing either URDF, validate the pit using a four-foot support proxy:

```text
             correct robot mass
          +----------------------+
          |   rigid proxy body   |
          +----------------------+
             |    |    |    |
            pad  pad  pad  pad
                 SCM terrain
```

The proxy body should expose configurable:

- total mass;
- payload mass;
- stance length and width;
- foot radius or area;
- body center-of-mass position.

Once this model settles correctly, use the URDF only to replace its appearance and refine mass distribution.

---

## 5. Candidate poses

Define only two candidate base poses initially.

### Candidate A: close inspection pose on sand

- Better target visibility
- Shorter path
- Four feet on the SCM terrain
- Higher predicted sinkage and terrain disturbance

### Candidate B: rigid bypass pose

- Slightly farther from the target
- Longer approach
- Four feet on the rigid floor
- Negligible sinkage

The planner should combine a task cost and a support cost:

```latex
J(q,r)
=
w_v J_{\mathrm{view}}(q)
+
w_p J_{\mathrm{path}}(q)
+
w_s d_{\max}(q,r)
+
w_t \phi_{\mathrm{tilt}}(q,r)
+
w_u U(q,r).
```

For the visual prototype, `J_view` and `J_path` can be manually defined constants. The support terms should come from simulation.

Do not manually force Go1 and Spot to choose different poses. Adjust the terrain and cost weights until their simulated support outcomes produce the intended decision under one shared cost function.

---

## 6. Demo storyboard

Target duration: approximately `45-75 seconds`.

### Shot 1: establish the environment

- Orbit the camera around the rigid floor and central sand pit.
- Show the inspection target.
- Display the two candidate standing poses.
- Label them `Closer / deformable` and `Farther / rigid`.

### Shot 2: Go1 query

- Place a translucent Go1 model at both candidates.
- Display predicted support outcomes over the sand candidate.
- Use four foot-local circles colored by predicted settlement.
- Show a compact label such as:

```text
Go1
max sinkage: 8 mm
predicted tilt: 0.7 deg
selected: close sand pose
```

### Shot 3: Go1 execution

- Move or fade Go1 into the selected pose.
- Release the locked-stance model.
- Show the feet settling and the pit deforming.
- Freeze briefly on the final footprint and body attitude.

### Shot 4: reset and Spot query

- Reset the SCM terrain.
- Replace Go1 with Spot.
- Keep the camera, scene, target, candidate poses, terrain, and cost weights unchanged.
- Display Spot's predicted support outcomes.

```text
Spot
max sinkage: 24 mm
predicted tilt: 2.1 deg
selected: rigid pose
```

These numbers are placeholders. Use the simulator outputs in the final demo.

### Shot 5: Spot execution

- Move Spot to the rigid candidate.
- Show stable support with no meaningful terrain deformation.
- Optionally show a ghosted counterfactual Spot settling into the sand as an inset.

### Shot 6: closing comparison

Display the same map with two decisions:

```text
Go1  -> close sand pose
Spot -> rigid bypass pose
```

Closing message:

> Same environment. Different embodiment. Different physically viable plan.

---

## 7. Simulation states and outputs

### Input configuration

```yaml
world:
  floor_size_m: [3.0, 3.0]
  gravity_mps2: [0.0, 0.0, -9.81]
  timestep_s: 0.0005

pit:
  model: SCM
  size_m: [1.2, 1.2]
  grid_spacing_m: 0.01
  top_elevation_m: 0.0

robot:
  model: go1  # go1 | spot | proxy
  payload_kg: 0.0
  joint_mode: locked_stance

trial:
  candidate_pose: sand  # sand | rigid
  settle_time_s: 3.0
  reset_terrain: true
```

### Record for each trial

- robot name and payload;
- selected candidate pose;
- four initial foot positions;
- four final foot positions;
- maximum and mean foot sinkage;
- body roll and pitch;
- center-of-mass height change;
- initial terrain height map;
- loaded terrain height map;
- residual terrain height map after robot removal;
- simulation runtime;
- final screenshot and optional video.

### Support outcome

```python
@dataclass
class SupportOutcome:
    robot: str
    candidate_pose: str
    foot_sinkage_m: np.ndarray       # shape: [4]
    body_roll_rad: float
    body_pitch_rad: float
    com_height_change_m: float
    initial_heightmap_m: np.ndarray
    loaded_heightmap_m: np.ndarray
    residual_heightmap_m: np.ndarray
    runtime_s: float
```

---

## 8. Implementation modules

```text
quick_support_demo/
├── assets/
│   ├── go1/
│   ├── spot/
│   ├── target/
│   └── textures/
│
├── configs/
│   ├── world.yaml
│   ├── terrain.yaml
│   ├── go1.yaml
│   ├── spot.yaml
│   └── candidates.yaml
│
├── chrono_demo/
│   ├── build_world.py
│   ├── build_scm_pit.py
│   ├── build_support_proxy.py
│   ├── load_robot.py
│   ├── run_support_trial.py
│   ├── extract_heightmap.py
│   └── record_demo.py
│
├── planning/
│   ├── candidate_poses.py
│   ├── support_cost.py
│   └── select_pose.py
│
├── overlays/
│   ├── draw_candidates.py
│   ├── draw_foot_sinkage.py
│   └── compose_comparison.py
│
├── outputs/
│   ├── trials/
│   ├── screenshots/
│   └── videos/
│
└── run_demo.py
```

For the first pass, these modules may be combined into one script. Split them only after the scene runs correctly.

---

## 9. Build milestones

### Milestone 1: terrain contact

- [ ] Create rigid perimeter floor.
- [ ] Create central SCM patch.
- [ ] Drop one rigid circular platen onto the patch.
- [ ] Confirm visible, stable settlement.
- [ ] Reset terrain to the initial state.
- [ ] Extract pre-load and loaded height maps.

### Milestone 2: four-foot proxy

- [ ] Create a floating rigid body with four foot pads.
- [ ] Configure Go1 mass and stance.
- [ ] Configure Spot mass and stance.
- [ ] Record sinkage and body tilt.
- [ ] Verify that heavier loading produces a meaningfully different result.

### Milestone 3: visual robot assets

- [ ] Import Go1 URDF and meshes.
- [ ] Import Spot URDF and meshes.
- [ ] Apply nominal standing configurations.
- [ ] Substitute robust primitive foot collisions.
- [ ] Confirm mass, scale, orientation, and center of mass.

### Milestone 4: planning display

- [ ] Add two candidate markers.
- [ ] Add target visibility and path costs.
- [ ] Compute one shared support-aware objective.
- [ ] Display selected pose and predicted values.
- [ ] Record Go1 and Spot comparison.

### Milestone 5: Gaussian/MPM extension

- [ ] Add fixed RGB, depth and segmentation cameras.
- [ ] Export camera intrinsics and poses.
- [ ] Build the metric Gaussian map.
- [ ] Construct the custom physical Gaussian data structure.
- [ ] Calibrate Genesis MPM from Chrono platen observations.
- [ ] Replace SCM-oracle planning outcomes with Genesis MPM predictions.
- [ ] Transfer predicted MPM displacement back to the Gaussian map.

---

## 10. Acceptance criteria

The minimum visual demo is complete when:

1. One script can select `go1` or `spot` and reset the world.
2. Both robots appear at the correct metric scale.
3. A four-foot model produces stable contact on rigid terrain.
4. The same model produces visible settlement on the SCM patch.
5. Go1 and Spot support outcomes differ under the same terrain parameters.
6. One shared planning objective can produce different selected poses.
7. The final view clearly shows the robot, pit, target, candidate poses, and result.
8. The demo can be replayed deterministically from a fixed configuration.

The physical-GS extension is complete when the MPM-predicted deformed Gaussian map can render a post-contact depth image that is compared with the Chrono observation.

---

## 11. What to simplify and what not to fake

### Safe simplifications

- locked robot stance instead of locomotion;
- primitive foot collision shapes;
- two discrete candidate poses;
- manually specified view and path costs;
- SCM as the initial support oracle;
- offline support rollouts;
- kinematic movement to the final pose;
- oracle segmentation before adding learned segmentation.

### Do not fake

- robot mass;
- robot scale;
- stance and foot placement geometry;
- terrain deformation used by the support cost;
- one shared cost function across robot configurations;
- reported sinkage and tilt values;
- the difference between a Chrono SCM oracle result and a Genesis MPM prediction.

---

## 12. Recommended first deliverable

The first deliverable should be a deterministic split-screen video:

- **Left:** Go1 selects and settles at the sand pose.
- **Right:** Spot evaluates the same scene and selects the rigid pose.
- **Bottom overlay:** mass, maximum predicted sinkage, predicted tilt, and selected candidate.

Do not include Gaussian reconstruction in this first video unless it already works. The point of this deliverable is to establish that the environment, robot scaling, deformable support model, and embodiment-conditioned decision are visually legible.

The second video can replace the direct SCM support query with:

```text
simulated RGB-D -> Gaussian map -> Genesis MPM query
                  -> support decision -> Chrono execution
```

---

## References

- Project Chrono terrain models: <https://api.projectchrono.org/vehicle_terrain.html>
- Project Chrono sensor system: <https://api.projectchrono.org/sensor_overview.html>
- Project Chrono URDF parser: <https://api.projectchrono.org/classchrono_1_1parsers_1_1_ch_parser_u_r_d_f.html>
- Unitree Go1 robot description: <https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go1_description>
- Spot robot description: <https://github.com/bdaiinstitute/spot_description>
- PhysGaussian: <https://arxiv.org/abs/2311.12198>
