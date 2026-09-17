"""A simulated differential-drive robot node: subscribes to /cmd_vel,
integrates simple unicycle kinematics, and publishes its updated pose on
/odom -- closing the control loop with the planner and controller nodes.
"""

from __future__ import annotations

import math
from typing import Tuple

from .controller import normalize_angle
from .core import Node, TopicRegistry
from .messages import Pose2D, Twist


def integrate_pose(pose: Pose2D, linear: float, angular: float, dt: float) -> Pose2D:
    """Forward-Euler integration of unicycle/differential-drive kinematics:

        x' = x + v*cos(theta)*dt
        y' = y + v*sin(theta)*dt
        theta' = theta + w*dt

    Standalone and pure so it is trivial to unit test against known inputs.
    """
    new_x = pose.x + linear * math.cos(pose.theta) * dt
    new_y = pose.y + linear * math.sin(pose.theta) * dt
    new_theta = normalize_angle(pose.theta + angular * dt)
    return Pose2D(x=new_x, y=new_y, theta=new_theta)


class DiffDriveRobotNode(Node):
    """Simulated robot base. Holds the last /cmd_vel it received and
    integrates its pose by `dt` every spin iteration, republishing /odom."""

    def __init__(
        self,
        registry: TopicRegistry,
        start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        dt: float = 0.1,
        name: str = "robot_node",
    ):
        super().__init__(name, registry)
        self.dt = dt
        self.pose = Pose2D(*start_pose)
        self.last_cmd = Twist(linear=0.0, angular=0.0)

        self.odom_publisher = self.create_publisher(Pose2D, "/odom")
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel)
        self.create_timer(self.dt, self._integrate_step)

        # Publish the initial pose immediately so a controller node that is
        # already subscribed has a pose to work with before the first spin
        # iteration runs.
        self.odom_publisher.publish(self.pose)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd = msg

    def _integrate_step(self) -> None:
        self.pose = integrate_pose(self.pose, self.last_cmd.linear, self.last_cmd.angular, self.dt)
        self.odom_publisher.publish(self.pose)
