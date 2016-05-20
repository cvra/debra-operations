import zmqmsgbus
import argparse
import math
import numpy as np
import time


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
        self.max_heading_error = 0.2 # [rad]
        self.waypoints_speed = 0.1 # [m/s]
        self.heading_pid = PID(kp=2,ki=0.5,kd=0.1,ilimit=0,freq=self.frequency)
        self.distance_pid = PID(kp=250,ki=0,kd=0,ilimit=0,freq=self.frequency)

    def error(self, pose, target):
        distance_to_wp = pose.distance_to(target)
        heading_error = 0
        distance_error = -pose.forward_distance_to(target)
        distance_error = limit(distance_error, -self.waypoints_speed, self.waypoints_speed)

        if distance_to_wp > self.min_distance_error:
            bearing_to_wp = pose.abs_bearing_to(target)
            heading_error = periodic_error(pose.theta - bearing_to_wp)
            if abs(heading_error) > self.max_heading_error:
                distance_error = 0
        else:
            # arrived at taget; turn to target heading
            heading_error = periodic_error(pose.theta - target.theta)
            distance_error = -pose.forward_distance_to(target)

        return [distance_error, heading_error]

    def process(self, pose, target):
        distance_error, heading_error = self.error(pose, target)
        head_ctrl = self.heading_pid.process(heading_error)
        dist_ctrl = self.distance_pid.process(distance_error)
        v_left = dist_ctrl - head_ctrl
        v_right = dist_ctrl + head_ctrl
        return [v_left, v_right]


def odometry_msg_handler(pose, topic, msg):
    x, y, theta = msg
    pose.update([x,y], theta)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('target_x', type=float)
    parser.add_argument('target_y', type=float)
    parser.add_argument('target_theta', type=float)
    args = parser.parse_args()

    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    pose = RobotPose(xy=[0,0],theta=0)
    handler = lambda topic, msg: odometry_msg_handler(pose, topic, msg)
    node.register_message_handler('/position', handler)

    time.sleep(1)

    target_x = args.target_x
    target_y = args.target_y
    target_theta = args.target_theta

    waypoint = WayPoint()
    target = RobotPose(xy=[target_x, target_y], theta=target_theta)

    while True:
        v_left, v_right = waypoint.process(pose, target)
        node.call('/actuator/velocity', ['left-wheel', -v_left]) # left wheel velocity inversed
        node.call('/actuator/velocity', ['right-wheel', v_right])
        time.sleep(1/waypoint.frequency)

if __name__ == '__main__':
    main()
