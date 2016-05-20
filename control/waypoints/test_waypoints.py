import waypoints as wp
import unittest

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
