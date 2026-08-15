from __future__ import annotations


def import_chrono():
    try:
        import pychrono.core as chrono
        import pychrono.vehicle as veh
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "PyChrono with vehicle bindings is required. Use: "
            "conda activate chrono_splat"
        ) from exc
    return chrono, veh

