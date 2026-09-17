import unittest

from ros2_nav_sim.costmap import BUNDLED_MAP_LINES, Costmap


class TestCostmap(unittest.TestCase):
    def test_from_ascii_marks_hash_as_occupied_and_dot_as_free(self):
        lines = [
            ".#.",
            "..#",
            "#..",
        ]
        costmap = Costmap.from_ascii(lines)

        self.assertFalse(costmap.is_occupied((0, 0)))
        self.assertTrue(costmap.is_occupied((0, 1)))
        self.assertFalse(costmap.is_occupied((0, 2)))
        self.assertTrue(costmap.is_occupied((2, 0)))
        self.assertTrue(costmap.is_free((0, 0)))
        self.assertFalse(costmap.is_free((0, 1)))

    def test_bundled_map_start_and_goal_cells_are_free(self):
        from ros2_nav_sim.costmap import DEFAULT_GOAL, DEFAULT_START

        costmap = Costmap.bundled_map()
        self.assertTrue(costmap.is_free(DEFAULT_START))
        self.assertTrue(costmap.is_free(DEFAULT_GOAL))

    def test_bundled_map_wall_cells_are_occupied(self):
        costmap = Costmap.bundled_map()
        # Row 3 of BUNDLED_MAP_LINES is "####.#####": columns 0-3 occupied,
        # column 4 free, columns 5-9 occupied.
        self.assertEqual(BUNDLED_MAP_LINES[3], "####.#####")
        for col in [0, 1, 2, 3, 5, 6, 7, 8, 9]:
            self.assertTrue(costmap.is_occupied((3, col)))
        self.assertTrue(costmap.is_free((3, 4)))

    def test_out_of_bounds_cell_is_not_free(self):
        costmap = Costmap.bundled_map()
        self.assertFalse(costmap.is_free((-1, 0)))
        self.assertFalse(costmap.is_free((0, 999)))
        self.assertFalse(costmap.in_bounds((100, 100)))

    def test_neighbors4_excludes_occupied_and_out_of_bounds_cells(self):
        grid = [
            [0, 1],
            [0, 0],
        ]
        costmap = Costmap(grid)
        # (0,0)'s 4-neighbors are (-1,0) [OOB], (1,0) [free], (0,-1) [OOB],
        # (0,1) [occupied]. Only (1,0) should come back.
        neighbors = costmap.neighbors4((0, 0))
        self.assertEqual(neighbors, [(1, 0)])

    def test_grid_to_world_uses_cell_center_and_resolution(self):
        grid = [[0, 0], [0, 0]]
        costmap = Costmap(grid, resolution=2.0)
        x, y = costmap.grid_to_world((0, 0))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.0)
        x, y = costmap.grid_to_world((1, 0))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 3.0)

    def test_rejects_ragged_grid(self):
        with self.assertRaises(ValueError):
            Costmap([[0, 0], [0]])


if __name__ == "__main__":
    unittest.main()
