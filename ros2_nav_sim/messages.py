"""Plain-dataclass stand-ins for the ROS2 message types this project needs
(geometry_msgs/Pose2D-ish, geometry_msgs/Twist, nav_msgs/Path,
nav_msgs/OccupancyGrid-ish). Real ROS2 messages are generated from .msg/.idl
definitions and serialized over DDS; these are just Python objects passed by
reference in-process. Field names deliberately echo the real ROS2 message
fields so the mapping is obvious.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Pose2D:
    """Echoes geometry_msgs/msg/Pose2D."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians


@dataclass
class Twist:
    """Echoes geometry_msgs/msg/Twist, flattened to the 2D fields this
    differential-drive robot actually uses (linear.x and angular.z)."""

    linear: float = 0.0
    angular: float = 0.0


@dataclass
class Path:
    """Echoes nav_msgs/msg/Path, flattened to a plain list of (x, y) world
    coordinates instead of a list of full PoseStamped."""

    poses: List[Tuple[float, float]] = field(default_factory=list)


@dataclass
class OccupancyGrid:
    """Echoes nav_msgs/msg/OccupancyGrid: a flattened, row-major grid of
    cell costs (0 = free, 100 = occupied), plus its dimensions and the
    world-space size of one cell."""

    width: int = 0
    height: int = 0
    resolution: float = 1.0
    data: List[int] = field(default_factory=list)
