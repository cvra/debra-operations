import zmqmsgbus
import argparse
import math
import numpy as np
import time
import copy
from threading import Lock

OBSTACLE_MIN_DISTANCE = 0.4 # [m]
OBSTACLE_TIMEOUT = 0.3 # [s]

class RobotPose:
    def __init__(self, xy, theta):
        self.update(xy, theta)

    def distance_to(self, target):
        return np.linalg.norm(target.xy - self.xy)

    def abs_bearing_to(self, target):
        d = target.xy - self.xy
        return math.atan2(d[1], d[0])

    def rel_bearing_to(self, target):
        b = self.abs_bearing_to(target)
        return b - self.theta

    def forward_distance_to(self, target):
        d = target.xy - self.xy
        h = [math.cos(self.theta), math.sin(self.theta)]
        return np.dot(d, h)

    def update(self, xy, theta):
        self.xy = np.array(xy)
        self.theta = theta


def periodic_error(angle):
    angle = math.fmod(angle, 2*math.pi)
    if angle > math.pi:
        return angle - 2*math.pi
    if angle < -math.pi:
        return angle + 2*math.pi
    return angle


def limit(val, min, max):
    if val < min:
        return min
    if val > max:
        return max
    return val


class binaryHysteresis:
    def __init__(self, activate, deactivate):
        self.activate = activate
        self.deactivate = deactivate
        self.activated = False

    def evaluate(self, val):
        if val > self.activate:
            self.activated = True
        if val < self.deactivate:
            self.activated = False
        return self.activated


class PID:
    def __init__(self, kp, ki, kd, ilimit, freq):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ilimit = ilimit
        self.frequency = freq
        self.integrator = 0
        self.previous_error = 0

    def process(self, error):
        self.integrator += error
        # limit
        self.integrator = min(self.integrator, self.ilimit)
        self.integrator = max(self.integrator, -self.ilimit)

        output  = - self.kp * error
        output += - self.ki * self.integrator / self.frequency
        output += - self.kd * (error - self.previous_error) * self.frequency

        self.previous_error = error
        return output


class WayPoint:
    def __init__(self):
        self.frequency = 50 # [Hz]
        self.min_distance_error = 0.05 # [m]
        self.waypoints_distance_limit = 0.4 # [m]
        self.rotation_speed = 0.5 # [rad/s]
        self.heading_pid = PID(kp=15,ki=0.,kd=1,ilimit=0,freq=self.frequency)
        self.distance_pid = PID(kp=80,ki=0,kd=10,ilimit=0,freq=self.frequency)
        self.heading_error_large = binaryHysteresis(activate=0.4, deactivate=0.1)

    def error(self, pose, target):
        bearing_to_wp = pose.abs_bearing_to(target)
        heading_error = periodic_error(pose.theta - bearing_to_wp)
        distance_error = -pose.forward_distance_to(target)
        distance_error = limit(distance_error, -self.waypoints_distance_limit, self.waypoints_distance_limit)

        if pose.distance_to(target) > self.min_distance_error:
            if self.heading_error_large.evaluate(abs(heading_error)):
                distance_error = 0
        else:
            # arrived at taget; turn to target heading if specified
            if target.theta is None:
                heading_error = 0
            else:
                heading_error = periodic_error(pose.theta - target.theta)

        heading_error = limit(heading_error, -self.rotation_speed, self.rotation_speed)
        return [distance_error, heading_error]

    def process(self, pose, target):
        distance_error, heading_error = self.error(pose, target)
        head_ctrl = self.heading_pid.process(heading_error)
        dist_ctrl = self.distance_pid.process(distance_error)
        v_left = dist_ctrl - head_ctrl
        v_right = dist_ctrl + head_ctrl
        return [v_left, v_right]


def obstacle_avoidance_robot_should_stop(robot_pos, target, obstacles):
    fwd = target.xy - robot_pos.xy
    fwd = fwd / np.linalg.norm(fwd)
    for xy in obstacles:
        o = np.array(xy)
        obstacle_vec = o - robot_pos.xy
        obstacle_dist = np.linalg.norm(robot_pos.xy - o)
        if np.dot(obstacle_vec, fwd) > 0 and obstacle_dist < OBSTACLE_MIN_DISTANCE:
            return True
    return False



class ObstacleList:
    class Obstacle:
        def __init__(self, pos, time):
            self.pos = pos
            self.time = time

    def __init__(self):
        self.lock = Lock()
        self.lst = []

    def add(self, xy, now):
        with self.lock:
            o = self.Obstacle(copy.copy(xy), now)
            self.lst.append(o)

    def remove_old(self, now):
        with self.lock:
            for ob in self.lst:
                if ob.time + OBSTACLE_TIMEOUT < now:
                    self.lst.remove(ob)

    def get(self, now):
        lst = list()
        with self.lock:
            for o in self.lst:
                lst.append(copy.copy(o.pos))
            return lst

def odometry_msg_handler(topic, msg):
    global pose
    global pose_lock
    x, y, theta = msg
    with pose_lock:
        pose.update([x,y], theta)

def waypoint_msg_handler(topic, msg):
    global target
    global target_lock
    with target_lock:
        if msg is not None:
            x, y, theta = msg
            if target is None:
                target = RobotPose([x, y], theta)
            else:
                target.update([x, y], theta)
        else:
            target = None

def main():
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    global pose
    global pose_lock
    pose = RobotPose(xy=[0,0],theta=0)
    pose_lock = Lock()
    node.register_message_handler('/position', odometry_msg_handler)

    global target
    global target_lock
    target = None
    target_lock = Lock()
    node.register_message_handler('/waypoint', waypoint_msg_handler)

    obstacle_list = ObstacleList()
    handler = lambda topic, msg: obstacle_list.add(msg, time.time())
    node.register_message_handler('/obstacle', handler)

    time.sleep(1)

    waypoint = WayPoint()

    while True:
        with target_lock:
            tg = copy.copy(target)

        if tg is not None:
            with pose_lock:
                pos = copy.copy(pose)

            obstacles = obstacle_list.get(time.time())
            if obstacle_avoidance_robot_should_stop(pos, tg, obstacles):
                v_left, v_right = 0, 0
            else:
                v_left, v_right = waypoint.process(pos, tg)

            node.call('/actuator/velocity', ['left-wheel', -v_left]) # left wheel velocity inversed
            node.call('/actuator/velocity', ['right-wheel', v_right])

        obstacle_list.remove_old(time.time())
        time.sleep(1/waypoint.frequency)

if __name__ == '__main__':
    main()
