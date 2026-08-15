from __future__ import annotations

import unittest

import numpy as np

from quick_support_demo.overlays.pyvista_renderer import soil_point_colors, surface_mesh_arrays


class SurfaceMeshArraysTest(unittest.TestCase):
    def test_regular_grid_connectivity(self) -> None:
        heightmap = np.arange(12, dtype=float).reshape(3, 4) / 1000.0

        points, faces = surface_mesh_arrays(heightmap)

        self.assertEqual(points.shape, (12, 3))
        self.assertEqual(faces.shape, (6, 4))
        np.testing.assert_allclose(points[:, 2], heightmap.ravel())
        np.testing.assert_array_equal(faces[0], [0, 1, 5, 4])
        np.testing.assert_array_equal(faces[-1], [6, 7, 11, 10])

    def test_soil_colors_are_deterministic_and_darkened_by_sinkage(self) -> None:
        level = np.zeros((5, 5), dtype=float)
        deformed = level.copy()
        deformed[2, 2] = -0.05

        first = soil_point_colors(level)
        second = soil_point_colors(level)
        deformed_colors = soil_point_colors(deformed)

        np.testing.assert_array_equal(first, second)
        self.assertEqual(first.dtype, np.uint8)
        self.assertLess(deformed_colors[12].mean(), first[12].mean())


if __name__ == "__main__":
    unittest.main()
