"""Wires together the planner, controller, and robot nodes on the bundled
map and runs the navigation loop end to end via Executor.spin().

Run directly with:

    python -m ros2_nav_sim.demo
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .controller import PurePursuitControllerNode
from .core import Executor, TopicRegistry
from .costmap import DEFAULT_GOAL, DEFAULT_START, Costmap
from .planner import PlannerNode
from .robot import DiffDriveRobotNode


@dataclass
class DemoResult:
    reached_goal: bool
    iterations_run: int
    max_iterations: int
    start_cell: Tuple[int, int]
    goal_cell: Tuple[int, int]
    path_cells: List[Tuple[int, int]]
    final_pose: Tuple[float, float, float]
    goal_world: Tuple[float, float]
    final_distance_to_goal: float


def run_demo(
    start_cell: Tuple[int, int] = DEFAULT_START,
    goal_cell: Tuple[int, int] = DEFAULT_GOAL,
    max_iterations: int = 1000,
    goal_tolerance: float = 0.3,
    costmap: Optional[Costmap] = None,
) -> DemoResult:
    registry = TopicRegistry()
    if costmap is None:
        costmap = Costmap.bundled_map(resolution=1.0)

    planner = PlannerNode(registry, costmap, start_cell, goal_cell)
    if not planner.path_found:
        raise RuntimeError(f"No path found from {start_cell} to {goal_cell} on the bundled map")

    start_world = costmap.grid_to_world(start_cell)
    goal_world = costmap.grid_to_world(goal_cell)

    controller = PurePursuitControllerNode(registry, goal_tolerance=goal_tolerance)
    robot = DiffDriveRobotNode(registry, start_pose=(start_world[0], start_world[1], 0.0))

    executor = Executor()
    executor.add_node(controller)
    executor.add_node(robot)

    iterations_run = 0
    for _ in range(max_iterations):
        executor.spin_once()
        iterations_run += 1
        if controller.reached_goal:
            break

    final_distance = math.hypot(goal_world[0] - robot.pose.x, goal_world[1] - robot.pose.y)

    return DemoResult(
        reached_goal=controller.reached_goal or final_distance <= goal_tolerance,
        iterations_run=iterations_run,
        max_iterations=max_iterations,
        start_cell=start_cell,
        goal_cell=goal_cell,
        path_cells=planner.cells,
        final_pose=(robot.pose.x, robot.pose.y, robot.pose.theta),
        goal_world=goal_world,
        final_distance_to_goal=final_distance,
    )


def main() -> None:
    result = run_demo()
    print("ROS2-concept navigation simulation")
    print("===================================")
    print(f"Start cell: {result.start_cell}   Goal cell: {result.goal_cell}")
    print(f"A* path ({len(result.path_cells)} cells): {result.path_cells}")
    print(f"Goal (world coords): ({result.goal_world[0]:.2f}, {result.goal_world[1]:.2f})")
    print(
        f"Final pose: x={result.final_pose[0]:.2f}, y={result.final_pose[1]:.2f}, "
        f"theta={result.final_pose[2]:.2f} rad"
    )
    print(f"Iterations run: {result.iterations_run} / {result.max_iterations}")
    print(f"Final distance to goal: {result.final_distance_to_goal:.3f} m")
    if result.reached_goal:
        print("Result: GOAL REACHED")
    else:
        print("Result: GOAL NOT REACHED within iteration budget")


if __name__ == "__main__":
    main()
