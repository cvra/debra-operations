import waypoints as wp
import unittest
import math


class RobotPoseTest(unittest.TestCase):

    def test_distance_to(self):
        p = wp.RobotPose(xy=[1, 2], theta=0)
        t = wp.RobotPose(xy=[0, 0], theta=0)
        self.assertAlmostEqual(p.distance_to(t), math.sqrt(5))

    def test_abs_bearing_to(self):
        p = wp.RobotPose(xy=[0, 0], theta=0)
        t = wp.RobotPose(xy=[0, 1], theta=0)
        self.assertAlmostEqual(p.abs_bearing_to(t), math.pi/2)

    def test_rel_bearing_to(self):
        p = wp.RobotPose(xy=[0, 0], theta=0)
        t = wp.RobotPose(xy=[0, 1], theta=0)
        self.assertAlmostEqual(p.rel_bearing_to(t), math.pi/2)
        p = wp.RobotPose(xy=[0, 0], theta=math.pi/2)
        t = wp.RobotPose(xy=[0, 1], theta=0)
        self.assertAlmostEqual(p.rel_bearing_to(t), 0)


class WayPointTest(unittest.TestCase):
    def test_heading_control_sign(self):
        p = wp.RobotPose(xy=[0,0],theta=0)
        t = wp.RobotPose(xy=[0,0],theta=0.5)
        waypoint = wp.WayPoint()
        vl, vr = waypoint.process(pose=p, target=t)
        self.assertTrue(vl < 0)
        self.assertTrue(vr > 0)

    def test_dist_control_sign(self):
        p = wp.RobotPose(xy=[0,0],theta=0)
        t = wp.RobotPose(xy=[1,0],theta=0)
        waypoint = wp.WayPoint()
        vl, vr = waypoint.process(pose=p, target=t)
        self.assertTrue(vl > 0)
        self.assertTrue(vr > 0)

    def test_dont_advance_if_heading_error_is_too_large(self):
        p = wp.RobotPose(xy=[0,0],theta=0)
        t = wp.RobotPose(xy=[0,1],theta=0)
        waypoint = wp.WayPoint()
        dist, head = waypoint.error(pose=p, target=t)
        self.assertTrue(dist == 0)
