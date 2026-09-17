"""A* path planning over a Costmap, plus a Node that publishes the
computed path on the /path topic (mirrors a ROS2 global planner node
publishing nav_msgs/Path)."""

from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from .costmap import Costmap
from .core import Node, TopicRegistry
from .messages import Path

Cell = Tuple[int, int]


def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(costmap: Costmap, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    """Grid A* search using 4-connected moves and a Manhattan heuristic.

    Returns the list of cells from start to goal inclusive, or None if no
    path exists (e.g. the goal is walled off).
    """
    if not costmap.is_free(start) or not costmap.is_free(goal):
        return None
    if start == goal:
        return [start]

    open_heap: List[Tuple[int, int, Cell]] = []
    counter = 0
    heapq.heappush(open_heap, (0, counter, start))

    came_from: Dict[Cell, Cell] = {}
    g_score: Dict[Cell, int] = {start: 0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        for neighbor in costmap.neighbors4(current):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _manhattan(neighbor, goal)
                counter += 1
                heapq.heappush(open_heap, (f_score, counter, neighbor))

    return None


def _reconstruct_path(came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def cells_to_world(costmap: Costmap, cells: List[Cell]) -> List[Tuple[float, float]]:
    return [costmap.grid_to_world(c) for c in cells]


class PlannerNode(Node):
    """A ROS2-style planner node. On construction it runs A* once from
    `start` to `goal` on the given costmap and publishes the result as a
    Path message on /path -- mirroring a one-shot global planner that plans
    at startup (real ROS2 planners typically also replan on new goals; this
    demo only needs a single plan)."""

    def __init__(
        self,
        registry: TopicRegistry,
        costmap: Costmap,
        start: Cell,
        goal: Cell,
        name: str = "planner_node",
    ):
        super().__init__(name, registry)
        self.costmap = costmap
        self.start = start
        self.goal = goal
        self.publisher = self.create_publisher(Path, "/path")

        self.cells = astar(costmap, start, goal)
        self.path_found = self.cells is not None
        self.world_path: List[Tuple[float, float]] = (
            cells_to_world(costmap, self.cells) if self.cells else []
        )

        if self.path_found:
            self.publisher.publish(Path(poses=list(self.world_path)))
