import zmq
import zmqmsgbus
import argparse
import math
import time
import unittest

class RobotPose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def distance_to(self, target):
        return math.sqrt((target.x - self.x)**2 + (target.y - self.y)**2)

    def heading_to(self, target):
        return math.atan2(target.y - self.y, target.x - self.x)

    def update(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def periodic_error(angle):
    angle = math.fmod(angle, 2*math.pi)
    if angle > math.pi:
        return angle - 2*math.pi
    if angle < -math.pi:
        return angle + 2*math.pi
    return angle

class PID:
    def __init__(self, kp, ki, kd, i_limit, freq):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.frequency = freq
        self.integrator = 0
        self.previous_error = 0

    def process(self, error):
        self.integrator += error
        # limit
        self.integrator = min(self.integrator, self.i_limit)
        self.integrator = max(self.integrator, -self.i_limit)

        output  = - self.kp * error
        output += - self.ki * self.integrator / self.frequency
        output += - self.kd * (error - self.previous_error) * self.frequency

        self.previous_error = error
        return output

class WayPoint:
    def __init__(self):
        self.frequency = 50 # [Hz]
        self.min_distance_error = 10e-3 # [m]
        self.max_heading_error = 0.2
        self.waypoints_speed = 4
        self.heading_pid = PID(kp=5,ki=0,kd=0.1,ilimit=0,self.frequency)
        self.distance_pid = PID(kp=250,ki=0,kd=0,ilimit=0,self.frequency)

    def process(self, pose, target):
        if target is None:
            return [0, 0]
        distance_to_wp = pose.distance_to(target)
        heading_error = 0
        distance_error = 0

        if distance_to_wp > self.min_distance_error:
            heading_to_wp = pose.heading_to(target)
            heading_error = periodic_error(pose.theta - heading_to_wp)
            if heading_error < self.max_heading_error:
                # distance to the waypoint projected onto the heading error
                next_setpoint =  self.waypoints_speed / self.frequency
                distance_error = -math.cos(heading_error) * min(next_setpoint, distance_to_wp)
        else:
            # arrived at taget; turn to target heading
            heading_error = periodic_error(pose.theta - target.theta)

        head_ctrl = self.heading_pid.process(heading_error)
        # dist_ctrl = self.distance_pid.process(distance_error)
        dist_ctrl = -250*distance_error
        v_left = dist_ctrl - head_ctrl
        v_right = dist_ctrl + head_ctrl
        return [v_left, v_right]

class WayPointTest(unittest.TestCase):
    def test_heading_control_sign(self):
        pose = RobotPose(0,0,0)
        target = RobotPose(0,0,0.5)
        waypoint = WayPoint()
        waypoint.set_target(target)
        vl, vr = waypoint.process(pose)
        self.assertTrue(vl < 0)
        self.assertTrue(vr > 0)

    def test_dist_control_sign(self):
        pose = RobotPose(0,0,0)
        target = RobotPose(1,0,0)
        waypoint = WayPoint()
        waypoint.set_target(target)
        vl, vr = waypoint.process(pose)
        self.assertTrue(vl > 0)
        self.assertTrue(vr > 0)

    def test_dont_advance_if_heading_error_is_too_large(self):
        pose = RobotPose(0,0,0)
        # todo

def odometry_msg_handler(pose, topic, msg):
    x, y, theta = msg
    pose.update(x, y, theta)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('target_x', type=float)
    parser.add_argument('target_y', type=float)
    parser.add_argument('heading', type=float)
    args = parser.parse_args()


    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    pose = RobotPose(0,0,0)
    handler = lambda topic, msg: odometry_msg_handler(pose, topic, msg)
    node.register_message_handler('odometry_raw', handler)

    time.sleep(1)

    target_x = args.target_x
    target_y = args.target_y
    target_theta = args.heading

    waypoint = WayPoint()
    target = RobotPose(target_x, target_y, target_theta)

    while True:
        v_left, v_right = waypoint.process(pose, target)
        node.call('/actuator/velocity', ['left-wheel', -v_left]) # left wheel velocity inversed
        node.call('/actuator/velocity', ['right-wheel', v_right])
        time.sleep(1/waypoint.frequency)

if __name__ == '__main__':
    main()
