import math
import unittest

from ros2_nav_sim.controller import compute_control, normalize_angle
from ros2_nav_sim.messages import Pose2D


class TestPurePursuitControl(unittest.TestCase):
    def test_target_directly_ahead_yields_zero_angular_and_full_linear(self):
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        target = (5.0, 0.0)
        linear, angular = compute_control(pose, target, max_linear=0.6)
        self.assertAlmostEqual(angular, 0.0, places=6)
        self.assertAlmostEqual(linear, 0.6, places=6)

    def test_target_to_the_left_yields_positive_angular_turn_left(self):
        # Robot faces +x (theta=0). Target is directly to its left (+y).
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        target = (0.0, 5.0)
        linear, angular = compute_control(pose, target)
        self.assertGreater(angular, 0.0)

    def test_target_to_the_right_yields_negative_angular_turn_right(self):
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        target = (0.0, -5.0)
        linear, angular = compute_control(pose, target)
        self.assertLess(angular, 0.0)

    def test_target_behind_robot_still_turns_toward_it_correct_sign(self):
        # Robot faces +x. Target is behind and slightly to the left
        # (negative x, small positive y) -> desired heading is just past
        # +90 degrees, so the shortest turn from theta=0 is a left turn.
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        target = (-5.0, 1.0)
        linear, angular = compute_control(pose, target)
        self.assertGreater(angular, 0.0)
        # Large heading error -> the controller should slow way down.
        self.assertLess(linear, 0.1)

    def test_angular_command_is_clamped_to_max_angular(self):
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        target = (0.0, 0.001)  # ~90 degree error, tiny distance
        _, angular = compute_control(pose, target, k_angular=100.0, max_angular=1.5)
        self.assertLessEqual(angular, 1.5)
        self.assertGreaterEqual(angular, -1.5)

    def test_normalize_angle_wraps_into_range(self):
        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi, places=6)
        self.assertAlmostEqual(normalize_angle(-3 * math.pi), math.pi, places=6)
        self.assertAlmostEqual(normalize_angle(0.5), 0.5, places=6)


if __name__ == "__main__":
    unittest.main()
