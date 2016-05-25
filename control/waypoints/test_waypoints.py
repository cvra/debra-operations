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

    def test_forward_distance_to(self):
        p = wp.RobotPose(xy=[1, 1], theta=math.pi/2)
        t = wp.RobotPose(xy=[3, 2], theta=0)
        self.assertAlmostEqual(p.forward_distance_to(t), 1)
        p = wp.RobotPose(xy=[1, 1], theta=-math.pi/2)
        t = wp.RobotPose(xy=[3, 2], theta=0)
        self.assertAlmostEqual(p.forward_distance_to(t), -1)


class HelperTest(unittest.TestCase):

    def test_limit_min(self):
        self.assertEqual(wp.limit(-2, min=-1, max=1), -1)
        self.assertEqual(wp.limit(2, min=-1, max=1), 1)
        self.assertEqual(wp.limit(2, min=-1, max=3), 2)

    def test_hysteresis(self):
        h = wp.binaryHysteresis(0.6, 0.4)
        self.assertFalse(h.evaluate(0.1))
        self.assertFalse(h.evaluate(0.5))
        self.assertTrue(h.evaluate(0.7))
        self.assertTrue(h.evaluate(0.5))
        self.assertFalse(h.evaluate(0.3))



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


class ObstacleAvoidanceTest(unittest.TestCase):

    def test_obstacle_avoidance_robot_shoud_stop(self):
        p = wp.RobotPose(xy=[0,0],theta=0)
        t = wp.RobotPose(xy=[1,0],theta=0)
        self.assertFalse(wp.obstacle_avoidance_robot_should_stop(p, t, []))
        self.assertFalse(wp.obstacle_avoidance_robot_should_stop(p, t, [[0.5, 0.5]]))
        self.assertTrue(wp.obstacle_avoidance_robot_should_stop(p, t, [[0.1, 0.1]]))
        self.assertFalse(wp.obstacle_avoidance_robot_should_stop(p, t, [[-0.1, 0.1]]))

class ObstaclesTest(unittest.TestCase):
    def test_can_set_and_get(self):
        l = wp.ObstacleList()
        l.add([1,2], 1234)
        ob = l.get(1234)[0]
        self.assertEqual(ob, [1,2])

    def test_can_detect_old(self):
        l = wp.ObstacleList()
        l.add([1,2], 0)
        l.add([3,4], wp.OBSTACLE_TIMEOUT + 1)
        ob = l.get(wp.OBSTACLE_TIMEOUT + 1)
        self.assertEqual(ob[0], [3,4])

    def test_can_get_more_than_once(self):
        l = wp.ObstacleList()
        l.add([1,2], 0)
        l.add([3,4], wp.OBSTACLE_TIMEOUT + 1)
        ob = l.get(wp.OBSTACLE_TIMEOUT + 1)
        self.assertEqual(ob[0], [3,4])
        ob = l.get(wp.OBSTACLE_TIMEOUT + 1)
        self.assertEqual(ob[0], [3,4])
