import math
import unittest

from ros2_nav_sim.messages import Pose2D
from ros2_nav_sim.robot import integrate_pose


class TestDiffDriveIntegration(unittest.TestCase):
    def test_pure_forward_motion_moves_along_heading(self):
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        new_pose = integrate_pose(pose, linear=1.0, angular=0.0, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 1.0, places=6)
        self.assertAlmostEqual(new_pose.y, 0.0, places=6)
        self.assertAlmostEqual(new_pose.theta, 0.0, places=6)

    def test_pure_rotation_changes_heading_only(self):
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        new_pose = integrate_pose(pose, linear=0.0, angular=math.pi / 2, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 0.0, places=6)
        self.assertAlmostEqual(new_pose.y, 0.0, places=6)
        self.assertAlmostEqual(new_pose.theta, math.pi / 2, places=6)

    def test_forward_motion_at_45_degrees_moves_diagonally(self):
        pose = Pose2D(x=0.0, y=0.0, theta=math.pi / 4)
        new_pose = integrate_pose(pose, linear=2.0, angular=0.0, dt=1.0)
        expected = 2.0 * math.cos(math.pi / 4)
        self.assertAlmostEqual(new_pose.x, expected, places=6)
        self.assertAlmostEqual(new_pose.y, expected, places=6)

    def test_combined_linear_and_angular_over_small_dt(self):
        pose = Pose2D(x=1.0, y=1.0, theta=0.0)
        new_pose = integrate_pose(pose, linear=0.5, angular=0.1, dt=0.1)
        self.assertAlmostEqual(new_pose.x, 1.05, places=6)
        self.assertAlmostEqual(new_pose.y, 1.0, places=6)
        self.assertAlmostEqual(new_pose.theta, 0.01, places=6)

    def test_zero_velocity_leaves_pose_unchanged(self):
        pose = Pose2D(x=3.0, y=-2.0, theta=1.2)
        new_pose = integrate_pose(pose, linear=0.0, angular=0.0, dt=0.5)
        self.assertAlmostEqual(new_pose.x, 3.0, places=6)
        self.assertAlmostEqual(new_pose.y, -2.0, places=6)
        self.assertAlmostEqual(new_pose.theta, 1.2, places=6)


if __name__ == "__main__":
    unittest.main()
