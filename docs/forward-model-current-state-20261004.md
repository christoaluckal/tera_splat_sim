# Chrono-side forward-model status — 2026-10-04

Chrono is the synthetic GT generator for the fixed-domain Newton
terrain-deformation study. The accepted A0-A9 episodes are indexed in the
shared [historical evidence ledger](../../tera_splat/docs/newton/historical-evidence-20261004.md).

The current 2026-10-08 direction is an offline, mechanics-gated MPM prior with
a Chrono-supervised residual, Gaussian terrain update, and open-loop planner
evaluation. See the
[hybrid model design](../../tera_splat/docs/hybrid-chrono-mpm-residual-gaussian-planning-20261008.md).

Update, 2026-10-09: the initial static 35 mm proxy sentinel is complete.
Chrono accepted the 30.656 N center action at `0.349 s`, with `16.392 mm`
valid-interior peak depression. The unchanged Newton control failed its
trajectory-wide strict penetration gate, so the remaining proxy matrix is not
being generated yet. See the
[qualification report](../../tera_splat/docs/newton/static-proxy-qualification-20261009.md).

For every new episode, preserve the manifest's accepted time, the 0.25 s
residual duration, geometry, support mask, initial/loaded/residual maps, and
qualification gates. Current evidence supports bounded sim-to-sim analysis,
not real-terrain calibration or a generalized learned deformation predictor.
