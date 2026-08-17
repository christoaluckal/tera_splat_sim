from __future__ import annotations

import unittest

import numpy as np

from quick_support_demo.overlays.export_colmap_from_rgbd import (
    backproject_rgbd,
    encode_inverse_depth,
    opengl_c2w_to_colmap,
    qvec_to_rotmat,
    rotmat_to_qvec,
)


class ColmapPoseTest(unittest.TestCase):
    def test_opengl_look_direction_becomes_positive_colmap_z(self) -> None:
        c2w = np.eye(4)
        c2w[:3, 3] = [0.0, 0.0, 3.0]

        rotation, translation = opengl_c2w_to_colmap(c2w)
        target_camera = rotation @ np.zeros(3) + translation

        np.testing.assert_allclose(target_camera, [0.0, 0.0, 3.0])

    def test_rotation_quaternion_round_trip(self) -> None:
        angle = np.radians(37.0)
        rotation = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )

        np.testing.assert_allclose(qvec_to_rotmat(rotmat_to_qvec(rotation)), rotation)


class BackprojectionTest(unittest.TestCase):
    def test_center_pixel_backprojects_along_view_direction(self) -> None:
        depth = np.full((3, 3), np.nan, dtype=np.float32)
        depth[1, 1] = 2.0
        rgb = np.zeros((3, 3, 3), dtype=np.uint8)
        rgb[1, 1] = [10, 20, 30]
        intrinsics = {"fl_x": 1.0, "fl_y": 1.0, "cx": 1.0, "cy": 1.0}

        points, colors, pixels = backproject_rgbd(
            depth,
            rgb,
            np.eye(4),
            intrinsics,
            stride=1,
        )

        np.testing.assert_allclose(points, [[0.0, 0.0, -2.0]])
        np.testing.assert_array_equal(colors, [[10, 20, 30]])
        np.testing.assert_allclose(pixels, [[1.0, 1.0]])


class InverseDepthTest(unittest.TestCase):
    def test_encoding_matches_gaussian_splatting_loader(self) -> None:
        depth = np.array([[2.0, np.nan, 4.0, 0.0]], dtype=np.float32)

        encoded = encode_inverse_depth(depth, scale=1.0)
        decoded = encoded.astype(np.float32) / 65536.0

        np.testing.assert_allclose(decoded[0, [0, 2]], [0.5, 0.25], atol=2e-5)
        np.testing.assert_array_equal(encoded[0, [1, 3]], [0, 0])


if __name__ == "__main__":
    unittest.main()
