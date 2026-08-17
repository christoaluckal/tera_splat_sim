from __future__ import annotations

import unittest

import numpy as np

from quick_support_demo.overlays.splat_capture import (
    camera_to_world_opengl,
    metric_depth_to_uint16_mm,
    orbit_views,
    parse_float_list,
    parse_xyz,
    pinhole_intrinsics,
)


class OrbitViewsTest(unittest.TestCase):
    def test_multiple_theta_rings_cover_full_azimuth_without_duplicate_seam(self) -> None:
        views = orbit_views((15.0, 45.0), radius_m=2.0, target=(0.0, 0.0, 0.0), phi_count=4)

        self.assertEqual(len(views), 8)
        self.assertEqual([view.phi_deg for view in views[:4]], [0.0, 90.0, 180.0, 270.0])
        self.assertEqual([view.theta_deg for view in views[4:]], [45.0] * 4)
        np.testing.assert_allclose(np.linalg.norm(views[0].position), 2.0)

    def test_explicit_phi_angles_are_preserved_and_wrapped(self) -> None:
        views = orbit_views(
            (25.0,),
            radius_m=1.0,
            target=(1.0, 2.0, 3.0),
            phi_degrees=(-45.0, 20.0),
        )

        self.assertEqual([view.phi_deg for view in views], [315.0, 20.0])


class CameraMetadataTest(unittest.TestCase):
    def test_camera_to_world_is_rigid_and_looks_at_target(self) -> None:
        transform = camera_to_world_opengl((2.0, 0.0, 1.0), (0.0, 0.0, 0.0))

        np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1.0e-12)
        self.assertAlmostEqual(np.linalg.det(transform[:3, :3]), 1.0)
        np.testing.assert_allclose(transform[:3, 3], [2.0, 0.0, 1.0])
        expected_forward = np.array([-2.0, 0.0, -1.0]) / np.sqrt(5.0)
        np.testing.assert_allclose(-transform[:3, 2], expected_forward)

    def test_vertical_fov_intrinsics_use_square_pixels(self) -> None:
        intrinsics = pinhole_intrinsics(800, 600, 90.0)

        self.assertAlmostEqual(intrinsics["fl_x"], 300.0)
        self.assertAlmostEqual(intrinsics["fl_y"], 300.0)
        self.assertEqual(intrinsics["cx"], 400.0)
        self.assertEqual(intrinsics["cy"], 300.0)

    def test_cli_list_parsers(self) -> None:
        self.assertEqual(parse_float_list("10, 20.5,-3"), (10.0, 20.5, -3.0))
        self.assertEqual(parse_xyz("1,2,3"), (1.0, 2.0, 3.0))
        with self.assertRaises(ValueError):
            parse_xyz("1,2")

    def test_metric_depth_png_encoding(self) -> None:
        depth_m = np.array([[1.234, np.nan, 0.0, 100.0]], dtype=np.float32)

        encoded = metric_depth_to_uint16_mm(depth_m)

        np.testing.assert_array_equal(encoded, [[1234, 0, 0, 65535]])
        self.assertEqual(encoded.dtype, np.uint16)


if __name__ == "__main__":
    unittest.main()
