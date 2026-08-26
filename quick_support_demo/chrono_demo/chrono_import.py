from __future__ import annotations


def import_chrono():
    try:
        import pychrono.core as chrono
        import pychrono.vehicle as veh
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "PyChrono with vehicle bindings is required. The current chrono_splat "
            "environment has core bindings only; install or build a full PyChrono "
            "vehicle-enabled binding before running SCM workflows."
        ) from exc
    return chrono, veh

