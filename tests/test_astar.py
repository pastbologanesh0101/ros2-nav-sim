import unittest

from ros2_nav_sim.costmap import Costmap
from ros2_nav_sim.planner import astar


class TestAStar(unittest.TestCase):
    def test_finds_shortest_path_on_open_grid(self):
        # 5x5 open grid: Manhattan-optimal path from (0,0) to (4,4) has
        # length 9 cells (8 moves).
        grid = [[0] * 5 for _ in range(5)]
        costmap = Costmap(grid)

        path = astar(costmap, (0, 0), (4, 4))

        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 4))
        self.assertEqual(len(path), 9)  # optimal Manhattan distance + 1

    def test_finds_known_detour_around_wall(self):
        # 3x3 grid with the middle row blocked except one gap on the right
        # column, forcing a specific detour.
        # Row0: . . .
        # Row1: # # .
        # Row2: . . .
        grid = [
            [0, 0, 0],
            [1, 1, 0],
            [0, 0, 0],
        ]
        costmap = Costmap(grid)

        path = astar(costmap, (0, 0), (2, 0))

        self.assertIsNotNone(path)
        # The only way through is via column 2 (the gap in row 1).
        self.assertIn((0, 2), path)
        self.assertIn((1, 2), path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (2, 0))
        self.assertEqual(len(path), 7)  # known-shortest route length

    def test_reports_no_path_when_goal_is_walled_off(self):
        # Goal cell (2, 2) is completely surrounded by occupied cells -
        # unreachable from the start regardless of route.
        grid = [
            [0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0],
        ]
        costmap = Costmap(grid)

        path = astar(costmap, (0, 0), (2, 2))

        self.assertIsNone(path)

    def test_start_equals_goal_returns_single_cell_path(self):
        grid = [[0, 0], [0, 0]]
        costmap = Costmap(grid)

        path = astar(costmap, (0, 0), (0, 0))

        self.assertEqual(path, [(0, 0)])

    def test_start_or_goal_occupied_returns_none(self):
        grid = [[1, 0], [0, 0]]
        costmap = Costmap(grid)

        self.assertIsNone(astar(costmap, (0, 0), (1, 1)))

    def test_bundled_map_has_a_valid_path_from_default_start_to_goal(self):
        from ros2_nav_sim.costmap import DEFAULT_GOAL, DEFAULT_START

        costmap = Costmap.bundled_map()
        path = astar(costmap, DEFAULT_START, DEFAULT_GOAL)

        self.assertIsNotNone(path)
        self.assertEqual(path[0], DEFAULT_START)
        self.assertEqual(path[-1], DEFAULT_GOAL)
        # Every cell along the path must be free, and consecutive cells
        # must be 4-connected neighbors (no diagonal jumps, no teleporting).
        for cell in path:
            self.assertTrue(costmap.is_free(cell))
        for (r1, c1), (r2, c2) in zip(path, path[1:]):
            self.assertEqual(abs(r1 - r2) + abs(c1 - c2), 1)


if __name__ == "__main__":
    unittest.main()
