from __future__ import annotations

import unittest

from quick_support_demo.chrono_demo.hazard import RigidHazard, opposite_side_support_indices


class _Position:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


class RigidHazardTest(unittest.TestCase):
    def test_offset_foot_track_strikes_hazard(self) -> None:
        hazard = RigidHazard()

        self.assertTrue(hazard.intersects_foot(_Position(0.13, 0.0, 0.10), 0.035))

    def test_center_and_opposite_track_miss_hazard(self) -> None:
        hazard = RigidHazard()

        self.assertFalse(hazard.intersects_foot(_Position(0.0, 0.0, 0.10), 0.035))
        self.assertFalse(hazard.intersects_foot(_Position(-0.13, 0.0, 0.10), 0.035))

    def test_sufficient_clearance_avoids_strike(self) -> None:
        hazard = RigidHazard()

        self.assertFalse(hazard.intersects_foot(_Position(0.13, 0.0, 0.25), 0.035))

    def test_retained_support_is_opposite_world_x_hazard(self) -> None:
        offsets = [(0.2, 0.1, -0.3), (0.2, -0.1, -0.3), (-0.2, 0.1, -0.3), (-0.2, -0.1, -0.3)]

        self.assertEqual(opposite_side_support_indices(offsets, 0.13), {0, 2})
        self.assertEqual(opposite_side_support_indices(offsets, -0.13), {1, 3})


if __name__ == "__main__":
    unittest.main()
