# Overview and Status

Last verified: 2026-09-03

[Documentation index](README.md)

This repository has two distinct roles:

1. the quick-support robot/SCM demonstration;
2. generation and qualification of the Chrono SCM cylinder oracle consumed by
   the companion `tera_splat` forward-model calibration. Genesis is the frozen
   baseline; Newton integration is active on a separate branch.

The chronological overview through 2026-08-29 is archived in
[overview-and-status-through-2026-08-29.md](archive/overview-and-status-through-2026-08-29.md).
It is provenance, not an active task list.

## Quick-support demonstration

Implemented workflows include:

- stationary Go1 and Spot support proxies on SCM;
- open-loop Go1 traversal with independent contact feet;
- forward-turn-forward maneuver generation;
- a reduced-order rigid hazard skid/fall;
- rigid difficult terrain with commanded support-plane tilt;
- mesh-initialized rolling SCM terrain and signed DEM display;
- Matplotlib and PyVista/VTK rendering;
- calibrated final-state RGB-D orbit capture.

These workflows remain approximations. Traversal does not have an articulated
dynamical robot, connected leg forces, a state estimator, or a balance
controller. See [Limitations and Supported Claims](limitations-and-claims.md).

## Chrono oracle

The active cross-model oracle is:

`/data/christoa/Chrono/tera_splat_sim/validity_experiment/chrono_episodes/A0_oracle_guided_offset_5mm_gate6mm_v1`

| Item | Value |
| --- | --- |
| action | 1.5 kg cylinder; radius `73.025 mm`; height `50.8 mm` |
| center and constraint | `(0, +5 mm)`; vertical prismatic guide |
| SCM | `0.6 x 0.6 m`; `5 mm` spacing; `1 ms` timestep |
| loaded gate | below `6 mm/s` and `0.01 rad/s` for `0.10 s` |
| loaded time | `3.595 s` |
| residual duration | fixed `0.25 s` |
| valid support | `14,161` interior cells |
| cylinder sinkage | `34.270 mm` |

This episode is qualified and frozen. The low-speed loading rule is documented
as a sampling convention, not static equilibrium.

The legacy free-centered `A0_cal_full10mm` target is excluded from
calibration because its action and residual-time contract were incomplete. A
fresh replay proved its stored terrain was not corrupt; the exclusion is about
experiment qualification, not stale numerical data.

## Genesis bridge status

The companion repository now has an accepted promoted bed:

`/data/christoa/Chrono/tera_splat/outputs/validity_experiment/A0_oracle_guided_offset_5mm_gate6mm_prepared_5mm_n128_ratio_matched/prepared_bed`

| Item | Value |
| --- | ---: |
| particles | `307,461` |
| particle spacing/size | `5 mm` |
| MPM grid | `n128` |
| timestep | `0.5 ms` |
| particle spacings per cell | `3.125` |
| geostatic scale | `1.0` |
| preparation p99 speed | `0.492 mm/s` |
| H0 RMSE / maximum | `0.070 / 0.237 mm` |

The former n128 failure came from keeping 10 mm particles while halving the
MPM cell width. Restoring the accepted n64 particle-to-cell ratio with 5 mm
particles resolved initialization without changing physics or loosening gates.

## Previous best-known candidate

Before n128 promotion, the best valid coarse candidate was
`E=20 kPa`, `phi=18.149 deg`, `nu=0.100004` from W&B
`jg3b5v3s`.

### Observations

- Chrono sinkage: `34.270 mm`.
- Coarse Genesis sinkage: `34.051 mm`.
- Coarse objective: `8.548 mm`.
- Fresh unseeded study `e72xmaou` completed 12/12 valid observations but did
  not sample the low-`nu` corner and did not beat the incumbent.
- Anchor-inclusive study `vrxqwoe2` added nine valid observations and found
  nearby candidates at `8.605` and `8.643 mm`.

### Actions

1. froze this Chrono oracle and the fixed-time loss;
2. fixed the BayesOpt bootstrap particle-geometry constant;
3. built the ratio-matched 5 mm-particle/n128 bed;
4. kept geostatic scale 1.0 and all validity gates;
5. extended only the candidate preparation cap from 2 s to 4 s;
6. replayed the incumbent and two confirmations at identical times.

### Results

| Candidate | n64 objective | n128 objective | n128 loaded RMSE | n128 residual-footprint RMSE |
| --- | ---: | ---: | ---: | ---: |
| 20.000 kPa / 18.149 deg / 0.100004 | **`8.548 mm`** | **`9.626 mm`** | **`2.142 mm`** | **`14.966 mm`** |
| 18.110 kPa / 18.984 deg / 0.103989 | `8.605 mm` | `9.833 mm` | `2.188 mm` | `15.290 mm` |
| 20.186 kPa / 18.485 deg / 0.100693 | `8.643 mm` | `10.041 mm` | `2.316 mm` | `15.449 mm` |

The ordering survived resolution promotion. Compact n128 study `9on0s14j`
then improved the objective to `9.131 mm`, and lower-friction boundary study
`yab3idti` improved it again. Its iteration 011 candidate is now the confirmed
incumbent after exact replay `r2at0vvb`:

| Candidate | objective | loaded RMSE | residual-footprint RMSE | residual signed mean |
| --- | ---: | ---: | ---: | ---: |
| 20.433 kPa / 14.727 deg / 0.101895 | `8.704 mm` | `1.864 mm` | `13.678 mm` | `+12.941 mm` |

The exact replay passed candidate, phase, mask, score, and map-level
repeatability gates. Genesis still recovers too much and remains too high after
removal, but initialization and forward execution are consistent.

Retained-raw replay `ykep3esa` generated 78 sampled MPM rollout PLYs, complete
initial/loaded/residual particle states, aligned surface and raw-particle PCDs,
compressed comparison arrays, and loaded/residual isometric point-cloud plus
2D DEM-error figures. Its aggregate result was stable at `8.705 mm`. Four
residual cells, however, exceeded the frozen three-cell sparse projection-bin
allowance, so the run is visual evidence only and `r2at0vvb` remains the
authoritative confirmation.

The companion repository has now completed a non-learned diagnosis. Its 16
unique valid n128 candidates form a four-point loaded/residual Pareto front,
the incumbent recovery-error RMSE is `9.213 mm` inside the footprint, and
Genesis `F`/`Jp` changes localize plastic response around the action. A
controlled 2x2 matrix also found that halving Genesis timestep changes
residual-footprint RMSE by `+2.325 mm` at n64 and `+1.525 mm` at n128.
Resolution effects at fixed timestep are smaller (`+0.233` and `-0.566 mm`).
The follow-up same-state traces localize the preparation failure. At
`0.5/0.25/0.125 ms`, final p50/p95/p99 speeds are
`0.100/0.243/0.450`, `0.170/0.360/0.516`, and
`0.291/0.764/0.986 mm/s`. The dominant fastest population shifts from
wall/ground settling to almost entirely free-surface uplift; persistent-mover
median vertical displacement changes from `-3.135` to `+2.555 mm`.

## Current interpretation

Verified:

- Chrono target generation and timing contract;
- common-grid masked loaded/residual transfer;
- accepted full-state Genesis preparation;
- candidate-specific stress initialization;
- no-action stability testing;
- fixed-time scoring;
- online W&B BayesOpt;
- 5 mm-particle/n128 replay and map-level repeatability;
- retained raw particle states and aligned point-cloud/DEM-error export.
- Pareto, spatial/recovery, hidden-state, and numerical sensitivity diagnosis.
- same-state pre-settle speed, localization, and net-drift diagnosis.
- fresh Newton full-bed timestep/tolerance preparation diagnostics;
- continuous-state guided-cylinder loading/removal with initial, loaded, and
  residual raw/map output and explicit acceptance gates.

Not yet established:

- a Genesis parameter set that jointly matches loaded and residual deformation;
- Genesis timestep convergence for this complete preparation/rollout pipeline;
- held-out validation across load mass or position;
- agreement with measured real sand;
- Gaussian deformation transfer.
- Newton response timestep convergence and calibration;
- validated predictive Newton agreement beyond the one mechanics-qualified,
  uncalibrated response.

No further Chrono oracle change is indicated by the current residual mismatch
or by a downstream solver change.

## Next cross-model experiment

The compact studies and the non-learned diagnosis are complete. Before another
BayesOpt sweep, correct or ablate the Genesis containment/state-preparation
mechanism that produces timestep-dependent wall settling and fine-step surface
uplift. Then rerun the unchanged three-level preparation and response checks.
Keep the Chrono target, action, observation times, material, loss, and
acceptance rule unchanged while changing one numerical mechanism at a time.

The Newton branch preserves this same Chrono episode, action, mask,
loaded/residual times, and score definition. Its PIC preparation matrix passes
all timestep/tolerance, speed, H0, and map-consistency gates. Its full two-way
cylinder/removal run is finite and passes the strict zero-center-penetration
gate. Next establish response convergence across timestep before calibration
or a larger evaluation. The frozen check changes only timestep across
`0.5/0.25/0.125 ms`, compares loaded-minus-initial and residual-minus-initial
maps, and retains the existing preparation, finite/full-support, and strict
penetration gates. Its adjacent-level limits are `0.5 mm` DEM RMSE, `1.0 mm`
maximum DEM error, and `0.5 mm` loaded sinkage difference. Chrono fit is not a
numerical-convergence criterion. Saved particle arrays are archival output,
not solver restart checkpoints; Genesis state and observations are not
portable evidence.

Detailed operational rules live in
[`tera_splat/docs/chrono-oracle-run-contract.md`](../../tera_splat/docs/chrono-oracle-run-contract.md).
