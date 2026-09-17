"""ros2_nav_sim: a ROS2-CONCEPT node/topic/pub-sub simulation with a small
navigation stack (costmap + A* planner + pure-pursuit controller) built on top.

This package does NOT depend on ROS2 / rclpy / DDS. It re-implements the
*shape* of the ROS2 node graph API (Node, create_publisher,
create_subscription, Executor.spin) in plain Python so the architecture is
recognizable to anyone who has used real ROS2, without requiring a ROS2
install. See the top-level README.md for the full "why" and porting notes.
"""

__version__ = "0.1.0"
