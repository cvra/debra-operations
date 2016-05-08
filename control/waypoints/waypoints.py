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

    @classmethod
    def get(cls):
        global pose_x
        global pose_y
        global pose_theta
        return RobotPose(pose_x, pose_y, pose_theta)

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

WAYPOINTS_FREQUENCY = 50 # [Hz]
WAYPOINTS_MIN_DISTANCE_ERROR = 10e-3 # [m]
WAYPOINTS_MAX_HEADING_ERROR = 0.2

class WayPoint:
    def __init__(self):
        global WAYPOINTS_FREQUENCY
        self.heading_pid = PID(10,0,0,50,WAYPOINTS_FREQUENCY)
        self.distance_pid = PID(1,0,0,50,WAYPOINTS_FREQUENCY)
        self.target = None

    def set_target(self, target):
        self.target = target

    def process(self, pose):
        if self.target is None:
            return [0, 0]
        distance_to_wp = pose.distance_to(self.target)
        heading_error = 0
        distance_error = 0

        global WAYPOINTS_MIN_DISTANCE_ERROR
        if distance_to_wp > WAYPOINTS_MIN_DISTANCE_ERROR:
            heading_to_wp = pose.heading_to(self.target)
            heading_error = periodic_error(pose.theta - heading_to_wp)
            global WAYPOINTS_MAX_HEADING_ERROR
            if heading_error < WAYPOINTS_MAX_HEADING_ERROR:
                # distance to the waypoint projected onto the heading error
                distance_error = -math.cos(heading_error) * distance_to_wp
        else:
            # arrived at taget; turn to target heading
            heading_error = periodic_error(pose.theta - self.target.theta)

        head_ctrl = self.heading_pid.process(heading_error)
        dist_ctrl = self.distance_pid.process(distance_error)
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

pose_x = 0
pose_y = 0
pose_theta = 0

def odometry_msg_handler(topic, msg):
    global pose_x
    global pose_y
    global pose_theta
    pose_x, pose_y, pose_theta = msg

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('target_x', type=float)
    parser.add_argument('target_y', type=float)
    parser.add_argument('heading', type=float)
    args = parser.parse_args()


    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    node.register_message_handler('odometry_raw', odometry_msg_handler)

    time.sleep(1)

    x, y, theta = pose_x, pose_y, pose_theta
    target_x = x + args.target_x * math.cos(theta) - args.target_y * math.sin(theta)
    target_y = y + args.target_x * math.sin(theta) + args.target_y * math.cos(theta)
    target_theta = periodic_error(theta + args.heading)

    waypoint = WayPoint()
    target = RobotPose(target_x, target_y, target_theta)
    waypoint.set_target(target)

    while True:
        pose = RobotPose.get()
        v_left, v_right = waypoint.process(pose)
        print(v_left, v_right, '\t', pose.x, pose.y, pose.theta)
        node.call('/actuator/velocity', ['left-wheel', -v_left]) # left wheel velocity inversed
        node.call('/actuator/velocity', ['right-wheel', v_right])
        time.sleep(1/WAYPOINTS_FREQUENCY)

if __name__ == '__main__':
    main()
