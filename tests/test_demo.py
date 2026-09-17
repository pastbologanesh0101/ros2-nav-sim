import unittest

from ros2_nav_sim.costmap import DEFAULT_GOAL, DEFAULT_START
from ros2_nav_sim.demo import run_demo


class TestEndToEndDemo(unittest.TestCase):
    def test_robot_reaches_goal_within_iteration_budget(self):
        result = run_demo(max_iterations=1000)

        self.assertTrue(result.reached_goal)
        self.assertLess(result.iterations_run, 1000)
        self.assertLessEqual(result.final_distance_to_goal, 0.3 + 1e-6)

    def test_demo_plans_a_path_matching_default_start_and_goal(self):
        result = run_demo(max_iterations=1000)

        self.assertEqual(result.start_cell, DEFAULT_START)
        self.assertEqual(result.goal_cell, DEFAULT_GOAL)
        self.assertEqual(result.path_cells[0], DEFAULT_START)
        self.assertEqual(result.path_cells[-1], DEFAULT_GOAL)
        self.assertGreater(len(result.path_cells), 1)

    def test_demo_raises_when_goal_is_unreachable(self):
        # A goal cell fully boxed in by occupied cells has no path from the
        # start cell, regardless of route -- run_demo should surface that
        # as a clear error rather than silently doing nothing.
        from ros2_nav_sim.costmap import Costmap
        from ros2_nav_sim.planner import astar

        boxed_grid = [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ]
        boxed_costmap = Costmap(boxed_grid)
        # Sanity: confirm astar itself returns None for this pair first.
        self.assertIsNone(astar(boxed_costmap, (0, 0), (1, 1)))

        with self.assertRaises(RuntimeError):
            run_demo(start_cell=(0, 0), goal_cell=(1, 1), max_iterations=10, costmap=boxed_costmap)


if __name__ == "__main__":
    unittest.main()
