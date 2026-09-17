"""A simple pure-pursuit-style path-following controller.

`compute_control` is a standalone, easily-unit-testable function: given the
robot's current pose and a single target waypoint, it returns the
(linear, angular) velocity command that steers the robot toward that point.
`PurePursuitControllerNode` wraps it as a ROS2-style node that subscribes to
/path and /odom and publishes /cmd_vel once per spin iteration.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .core import Node, TopicRegistry
from .messages import Path, Pose2D, Twist


def normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


def compute_control(
    pose: Pose2D,
    target: Tuple[float, float],
    max_linear: float = 0.6,
    k_angular: float = 2.0,
    max_angular: float = 2.0,
) -> Tuple[float, float]:
    """Pure-pursuit-style control law for a single look-ahead target point.

    Returns (linear, angular). angular > 0 means turn left (counter-
    clockwise, standard math convention with theta measured from +x axis);
    angular < 0 means turn right.
    """
    dx = target[0] - pose.x
    dy = target[1] - pose.y
    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - pose.theta)

    angular = max(-max_angular, min(max_angular, k_angular * heading_error))
    # Drive forward at full speed when roughly facing the target; slow down
    # (and eventually stop translating, favoring an in-place turn) the more
    # the robot needs to turn.
    linear = max_linear * max(0.0, math.cos(heading_error))

    return linear, angular


class PurePursuitControllerNode(Node):
    """Subscribes to /path (planned waypoints) and /odom (current pose),
    and publishes /cmd_vel to steer along the path."""

    def __init__(
        self,
        registry: TopicRegistry,
        waypoint_tolerance: float = 0.4,
        goal_tolerance: float = 0.3,
        max_linear: float = 0.6,
        max_angular: float = 2.0,
        k_angular: float = 2.5,
        name: str = "controller_node",
    ):
        super().__init__(name, registry)
        self.waypoint_tolerance = waypoint_tolerance
        self.goal_tolerance = goal_tolerance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.k_angular = k_angular

        self.path: List[Tuple[float, float]] = []
        self.pose: Optional[Pose2D] = None
        self.waypoint_index = 0
        self.reached_goal = False

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel")
        self.create_subscription(Path, "/path", self._on_path)
        self.create_subscription(Pose2D, "/odom", self._on_odom)
        self.create_timer(0.1, self._control_step)

    def _on_path(self, msg: Path) -> None:
        self.path = list(msg.poses)
        self.waypoint_index = 0

    def _on_odom(self, msg: Pose2D) -> None:
        self.pose = msg

    def _control_step(self) -> None:
        if self.pose is None or not self.path or self.reached_goal:
            return

        goal = self.path[-1]
        dist_to_goal = math.hypot(goal[0] - self.pose.x, goal[1] - self.pose.y)
        if dist_to_goal <= self.goal_tolerance:
            self.reached_goal = True
            self.cmd_vel_publisher.publish(Twist(linear=0.0, angular=0.0))
            return

        # Advance the look-ahead index while we're within tolerance of the
        # current target waypoint (and it isn't the final goal waypoint).
        while (
            self.waypoint_index < len(self.path) - 1
            and math.hypot(
                self.path[self.waypoint_index][0] - self.pose.x,
                self.path[self.waypoint_index][1] - self.pose.y,
            )
            <= self.waypoint_tolerance
        ):
            self.waypoint_index += 1

        target = self.path[self.waypoint_index]
        linear, angular = compute_control(
            self.pose,
            target,
            max_linear=self.max_linear,
            k_angular=self.k_angular,
            max_angular=self.max_angular,
        )
        self.cmd_vel_publisher.publish(Twist(linear=linear, angular=angular))
