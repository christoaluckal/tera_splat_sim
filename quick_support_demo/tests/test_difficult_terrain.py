from __future__ import annotations

import unittest

import numpy as np

from quick_support_demo.chrono_demo.build_scm_pit import (
    SCMHeightCourse,
    build_rolling_scm_pit,
    build_scm_pit,
    sample_heightmap,
)
from quick_support_demo.chrono_demo.build_world import build_system
from quick_support_demo.chrono_demo.difficult_terrain import (
    DifficultCourse,
    RollingCourse,
    RigidPad,
    support_plane_attitude,
    terrain_adjusted_foot_targets,
)
from quick_support_demo.config import load_demo_config


class DifficultTerrainTest(unittest.TestCase):
    def setUp(self) -> None:
        self.feet = {
            "FR": np.array([0.2, -0.1, -0.3]),
            "FL": np.array([0.2, 0.1, -0.3]),
            "RR": np.array([-0.2, -0.1, -0.3]),
            "RL": np.array([-0.2, 0.1, -0.3]),
        }

    def test_height_at_uses_highest_overlapping_pad(self) -> None:
        course = DifficultCourse(
            pads=(RigidPad(0.0, 0.0, 0.4, 0.4, 0.05), RigidPad(0.0, 0.0, 0.2, 0.2, 0.09))
        )

        self.assertAlmostEqual(course.height_at(0.0, 0.0), 0.09)
        self.assertAlmostEqual(course.height_at(0.18, 0.0), 0.05)
        self.assertEqual(course.height_at(0.3, 0.0), 0.0)

    def test_right_side_pad_produces_bounded_roll(self) -> None:
        course = DifficultCourse(pads=(RigidPad(0.1, 0.0, 0.12, 1.0, 0.10),))

        roll, pitch, center_height = support_plane_attitude(
            self.feet, (0.0, 0.0), np.pi / 2.0, course, np.radians(14.0)
        )

        self.assertLess(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertGreater(center_height, 0.0)
        self.assertLessEqual(abs(roll), np.radians(14.0))

    def test_stance_foot_reaches_ground_after_pad(self) -> None:
        course = DifficultCourse(pads=(RigidPad(0.1, -0.2, 0.2, 0.2, 0.08),))
        body_position = np.array([0.0, 0.35, 0.305])

        targets = terrain_adjusted_foot_targets(
            self.feet,
            nominal_foot_z_m=-0.3,
            body_position_world=body_position,
            yaw_rad=np.pi / 2.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            course=course,
            foot_height_m=0.035,
        )

        front_right_world = body_position + targets["FR"]
        self.assertAlmostEqual(front_right_world[2], 0.0175)

    def test_swing_clearance_is_relative_to_local_surface(self) -> None:
        course = DifficultCourse(pads=(RigidPad(0.1, 0.2, 0.2, 0.2, 0.08),))
        feet = dict(self.feet)
        feet["FR"] = feet["FR"] + np.array([0.0, 0.0, 0.06])
        body_position = np.array([0.0, 0.0, 0.345])

        targets = terrain_adjusted_foot_targets(
            feet,
            nominal_foot_z_m=-0.3,
            body_position_world=body_position,
            yaw_rad=np.pi / 2.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            course=course,
            foot_height_m=0.035,
        )

        front_right_world = body_position + targets["FR"]
        self.assertAlmostEqual(front_right_world[2], 0.1575)

    def test_rolling_course_has_hills_valleys_and_flat_boundaries(self) -> None:
        course = RollingCourse()
        config = {"pit": {"size_m": [1.2, 1.2], "grid_spacing_m": 0.02}}
        heightmap = course.heightmap(config)

        self.assertGreater(float(np.max(heightmap)), 0.05)
        self.assertLess(float(np.min(heightmap)), -0.03)
        np.testing.assert_allclose(heightmap[[0, -1], :], 0.0, atol=1.0e-7)
        np.testing.assert_allclose(heightmap[:, [0, -1]], 0.0, atol=1.0e-7)

    def test_scm_mesh_initialization_preserves_rolling_relief(self) -> None:
        config = load_demo_config()
        config["terrain"]["pit"]["grid_spacing_m"] = 0.035
        system = build_system(config["world"])
        terrain = build_rolling_scm_pit(
            system,
            config["terrain"],
            RollingCourse(),
            visualization_mesh=False,
        )

        heightmap = sample_heightmap(terrain, config["terrain"])

        self.assertLess(float(np.min(heightmap)), -0.05)
        self.assertGreater(float(np.max(heightmap)), 0.07)
        self.assertLess(float(np.max(np.abs(heightmap[[0, -1], :]))), 0.01)

        course = SCMHeightCourse(terrain, (1.2, 1.2), outside_height_m=0.0)
        self.assertEqual(course.height_at(0.0, -1.1), 0.0)
        self.assertLess(course.height_at(0.0, 0.0), -0.03)

    def test_flat_scm_initialization_remains_zero(self) -> None:
        config = load_demo_config()
        config["terrain"]["pit"]["grid_spacing_m"] = 0.035
        system = build_system(config["world"])
        terrain = build_scm_pit(system, config["terrain"], visualization_mesh=False)

        heightmap = sample_heightmap(terrain, config["terrain"])

        np.testing.assert_allclose(heightmap, 0.0, atol=1.0e-7)


if __name__ == "__main__":
    unittest.main()
