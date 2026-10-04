# Chrono target-generation contract for forward models

Chrono SCM is the synthetic target generator, not a complete MPM state or a
claim of real-sand ground truth. Every episode must retain the initial,
loaded, and residual metric heightfields, valid support mask, action geometry,
accepted loaded timing, residual duration, soil parameters, and convergence
record.

The Newton comparison must use the episode's exact timing and reject any
candidate that fails finite support, guide, timing, or strict analytic
center-penetration gates. Valid episodes can then be compared on both surface
maps and deformation characteristics: peak/mean depression, footprint error,
depression volume, and centroid.

A2–A6 now provide a compact transfer suite. A5/A6 are mirrored spatial
holdouts and demonstrate why a single scalar RMSE is insufficient for judging
terrain-deformation prediction.
