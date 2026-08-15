# Physics and Terrain

[Documentation index](README.md)

## Chrono system

`build_system` creates a `ChSystemSMC`, selects the Bullet collision system, and
applies gravity from the world YAML. SMC is used because the contact material
and SCM terrain are evaluated in a compliant-contact simulation.

The rigid floor is not one plane under the terrain. Four fixed boxes form left,
right, near, and far margins around the `1.2 m` central opening. This prevents a
rigid surface at `z=0` from intercepting foot contacts intended for SCM.

## SCM construction

`build_scm_pit` performs the following operations:

1. creates `vehicle.SCMTerrain`;
2. sets its plane at the configured top elevation;
3. initializes patch dimensions and grid spacing;
4. passes the eight configured soil inputs to `SetSoilParameters`;
5. configures sinkage plotting for native Chrono visualization.

The current PyChrono API receives the friction angle value from the YAML as
degrees. The repository passes `28.0` directly. Historical code or documents
that convert this value to radians are not valid for the installed API.

## Height-map sampling

`sample_heightmap` generates regular `x` and `y` coordinates over the entire
pit, including both boundaries, and queries `terrain.GetHeight` at each node.
It validates finite values and the expected array shape.

The returned array convention is:

- shape: `(number_of_y_nodes, number_of_x_nodes)`;
- units: meters;
- rows increase with world `y`;
- columns increase with world `x`.

The outermost SCM query ring reports the patch base behavior rather than a
normal deforming surface node in this setup. Deformation metrics and DEM plots
therefore mask or omit that ring.

## Stationary support proxy

The stationary proxy is one rigid `ChBody` with:

- total robot plus payload mass;
- box inertia computed from the configured body dimensions;
- four box collision shapes at configured stance offsets;
- a contact material with friction `0.9`, Young's modulus `1e6`, and
  restitution `0.05`;
- either a Go1 triangle-mesh visual or a box visual.

The feet are collision shapes attached to the same body. This provides load
redistribution and body roll/pitch, but no leg compliance or joint motion.

## Stationary trial lifecycle

The body starts with configured clearance above `z=0`. The system advances with
the configured time step. A trial can finish before the time limit after:

- simulation time exceeds `0.25 s`;
- linear speed remains below `0.005 m/s`;
- angular speed remains below `0.01 rad/s`;
- both conditions persist for `0.10 s`.

After measuring the loaded state, the body is removed and SCM advances for
another `0.5 s` by default. The residual height map records terrain persistence
after unloading.

## Traversal contact approximation

Traversal uses four separate `ChBody` feet. Each body has a box collision shape
whose horizontal side length is chosen to match the area of the configured
circular foot:

```text
side = sqrt(pi * radius^2)
```

During stance:

- the foot is dynamic and collision-enabled;
- its planar position follows the desired gait target;
- its vertical position and velocity are left to Chrono;
- total robot mass is divided among the current stance feet.

During swing:

- the foot is fixed and collision-disabled;
- it follows the target swing trajectory;
- velocity and angular velocity are reset.

This produces discrete SCM loading and vertical settlement, but the feet do not
transmit forces to the trunk because no physical leg connects them.

## Deformation definitions

The 3D renderer reports active-node sinkage using interior nodes:

```text
sinkage = max(-current_height, 0)
active node threshold = 1e-5 m
mean = mean(active sinkage)
maximum = max(active sinkage)
```

The DEM panel uses:

```text
difference_mm = (current_height - initial_height) * 1000
```

Therefore:

- negative DEM values are subsidence;
- positive DEM values are uplift;
- zero means no elevation change.

## Sand behavior controls

The most influential controls are:

- `bekker_kphi`, `bekker_kc`: lower values generally increase penetration;
- `bekker_n`: changes nonlinear pressure-sinkage response;
- `mohr_cohesion`: lower cohesion makes the material less self-supporting;
- `mohr_friction_deg`: lower values reduce internal shear strength;
- `janosi_shear_m`: controls shear displacement buildup;
- `elastic_k`: lower values make elastic response softer;
- `damping_r`: changes transient settling and oscillation;
- foot area: smaller feet increase pressure;
- robot mass and payload: higher values increase load;
- grid spacing: affects spatial resolution and measured footprint shape;
- gait timing: changes which feet share load and how long each contact acts.

These knobs do not make SCM a grain-resolved DEM simulator. SCM is a continuum
soil-contact model and cannot show individual grains or avalanching particles.

## Resolution and numerical interpretation

The `35 mm` video smoke grid is useful for iteration but coarse relative to a
`35 mm` Go1 foot radius. Full resolution uses `10 mm` spacing and resolves local
footprints better, at significantly greater runtime.

Values should be compared only when all of these are held fixed:

- SCM parameters;
- robot/contact geometry;
- mass and payload;
- time step;
- grid spacing;
- loading sequence;
- deformation metric definition.

## Rigid hazard and reduced-order fall

`--hazard` selects a separate nondeforming course. It does not create an SCM
terrain object. Instead, it adds:

- a fixed `1.2 m x 1.2 m x 0.08 m` center plate;
- a fixed `0.18 m x 0.24 m` block;
- a configurable block center x offset and height.

The default block center is offset `0.13 m` toward the Go1 front-right/rear-right
foot track and is `0.13 m` high. A pure geometry test checks each foot center
against the block footprint and its bottom clearance against the block top. At
the first strike, the gait stops and independent feet are frozen with collision
disabled. The kinematic trunk is replaced in function by its existing rigid
support proxy: its body collision and two locked foot pads opposite the hazard
are enabled, while the two hazard-side pads were deliberately omitted. The
proxy contact friction is `0.18`.

The released proxy receives the commanded forward velocity, a lateral velocity
of `0.55 m/s` toward the hazard side, and a modest one-time tipping angular
velocity of `0.65 rad/s`. Chrono then owns gravity, rigid contact, translation,
and trunk attitude. In the verified run, the body skidded `0.332 m` laterally
before settling at `90.0 deg` tilt.

This is a reduced-order failure model: the trigger, one-sided support removal,
lateral speed, and initial tipping tendency are prescribed. The post-release
translation and rotation are not keyframed, but no articulated leg transmits
the obstacle force to the trunk. It is therefore unsuitable for articulated
stability or controller claims.

## Rigid difficult course

`--difficult-terrain` uses the same fixed `1.2 m x 1.2 m` center plate and adds
three fixed high-friction pads:

| Pad | Center `(x, y)` | Size `(x, y)` | Height |
|---|---|---|---:|
| right/near | `(0.13, -0.28) m` | `(0.30, 0.34) m` | `85 mm` |
| left/middle | `(-0.13, 0.12) m` | `(0.30, 0.34) m` | `70 mm` |
| right/far | `(0.08, 0.47) m` | `(0.44, 0.18) m` | `55 mm` |

At every physics step, the commanded foot x/y locations query the maximum pad
height beneath them. A least-squares plane

```text
z = slope_forward * x_body + slope_lateral * y_body + center_height
```

produces local roll `atan(slope_lateral)` and pitch
`-atan(slope_forward)`. Roll, pitch, and center elevation use a `0.14 s`
first-order response. Roll and pitch are independently clamped to the requested
maximum, `14 deg` by default. The verified course reaches `11.8 deg` resultant
trunk tilt while completing at world `y=0.95 m`.

Visual foot IK is resolved in world space after the trunk attitude update. For
each foot:

```text
stance center z = rigid surface height + 0.5 * foot height
swing center z  = stance center z + gait swing clearance
```

The resulting world target is transformed through the inverse tilted-trunk
rotation and passed back to analytical IK. This prevents a foot from retaining
the raised-pad Z coordinate after moving onto the base plate. In the side-view
reference, the front-right foot has `55.3 mm` bottom clearance during swing at
`4.50 s` and `0.0 mm` bottom clearance in stance at `5.50 s`.

This attitude is kinematic terrain following. Chrono owns the rigid course and
independent foot contacts, but it does not solve trunk balance or actuator
forces. There is no terrain deformation.

## Deformable rolling SCM course

`--rolling-terrain` uses a continuous `1.2 m x 1.2 m` analytical heightfield.
The surface combines a near hill, center valley, far hill, and lateral
cross-slope. An eighth-power boundary taper brings the height exactly to zero
at all four pit edges so the course joins the surrounding rigid floor without
a vertical step.

At the `35 mm` smoke sampling used for the verified video, Chrono's initialized
surface range is `-64.2 mm` to `+76.9 mm`. The sampled analytical grid is
triangulated into `ChTriangleMeshConnected`, then passed to the mesh overload
of `SCMTerrain.Initialize`. Chrono's mesh reference convention is compensated
by the lower relief extent so the tapered boundary remains near floor Z.

An `SCMHeightCourse` adapter queries the live `SCMTerrain.GetHeight` surface for
support-plane attitude and terrain-adjusted visual IK. The robot therefore
follows both the initial relief and its evolving contact depressions. Independent
Chrono foot bodies apply the nominal `12.5 kg` total load during stance.

The adapter applies SCM heights only inside `[-0.6, 0.6] m` in x and y. Outside
that box it returns the configured rigid-floor elevation, `0.0 m`. This keeps
the spawn and exit feet on the perimeter slab and avoids treating undefined
out-of-pit SCM queries as support heights.

The corrected forward-turn-forward run displaced 137 interior nodes with mean
subsidence `29.02 mm` and maximum subsidence `73.88 mm`. These values are direct
`initial_heightmap - final_heightmap` measurements on the coarse smoke grid;
they are not calibrated against a specific physical sand.
