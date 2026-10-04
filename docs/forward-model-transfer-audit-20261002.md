# Chrono oracle role in the forward-model audit

Chrono remains the synthetic target generator for the fixed bed/action
contract. The companion audit compares its loaded and residual heightfields
against mechanics-valid Newton predictions and reports not only map RMSE but
also peak/mean depression, footprint depression, depression volume, and
deformation centroid.

The completed A2–A5 cases show why these characteristics must be retained:
the 100 kPa Newton control underpredicts the Chrono depression volume in the
guided mass sweep, while the fitted incumbent reduces scalar surface error
but can overpredict local peaks and shift the spatial centroid. This is a
surrogate-transfer result, not real-sand validation.

The authoritative aggregate is
`tera_splat/diagnostics/forward_model_transfer_20261002/`; the rejected A2
incumbent is intentionally excluded because it failed the strict
center-penetration gate.
