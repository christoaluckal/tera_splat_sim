# A5 spatial-transfer addendum (2026-10-02)

Chrono episode `A5_oracle_mass1p625kg_lateral_x50mm_gate6mm_v1` is an
accepted `converged_speed_hold` oracle for the frozen 1.625 kg guided
cylinder at `(x=+0.05 m, y=0.0 m)`. Accepted loaded time is 0.816 s, loaded
sinkage is 22.188591 mm, and residual recovery is 0.25 s. Its 121×121 support
mask and initial terrain are bitwise identical to A4, with 14,161 valid cells
and finite initial/loaded/residual maps.

This is a spatial holdout: only the lateral action center changed; the
geometry, SCM, guide, soil, and timing contract remain fixed.

Newton results at exact timing (dt=0.25 ms, 8 proxy iterations, 16 rigid
substeps):

| candidate | valid/gated | objective | loaded RMSE | residual footprint RMSE |
|---|---|---:|---:|---:|
| frozen incumbent | yes | 3.379302 mm | 1.223777 mm | 4.311049 mm |
| unchanged control | yes | 3.272178 mm | 0.960535 mm | 4.623285 mm |

Both pass zero sampled center penetration and guide gates. The control is
slightly better on the scalar objective, while the incumbent better matches
the residual footprint; do not generalize incumbent superiority to all spatial
actions from this point alone.
