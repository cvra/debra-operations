import zmqmsgbus
import queue
import time
import numpy as np
import math


def periodic_error(angle):
    angle = math.fmod(angle, 2*math.pi)
    if angle > math.pi:
        return angle - 2*math.pi
    if angle < -math.pi:
        return angle + 2*math.pi
    return angle


class PositionEstimator():

    def __init__(self):
        self.position_odometry = np.array([0.0, 0.0])
        self.heading_odometry = 0
        self.position_offset = np.array([0.0, 0.0])
        self.heading_offset = 0

    def update_odometry(self, odometry):
        self.position_odometry = np.array(odometry[0:2])
        self.heading_odometry = odometry[2]

    def reset(self, pos=[0, 0], heading=0):
        self.position_offset = np.array(pos) - self.position_odometry
        self.heading_offset = heading - self.heading_odometry

    def update_lidar(self, lidar):
        lidar_heading = lidar[2]
        lidar_position = np.array(lidar[0:2]) - np.array([math.cos(lidar_heading), math.sin(lidar_heading)]) * 0.074
        delta_position = lidar_position - self.position_odometry - self.position_offset
        delta_heading = lidar_heading - self.heading_odometry - self.heading_offset
        delta_heading = periodic_error(delta_heading)
        if np.linalg.norm(delta_position) < 0.5 and abs(delta_heading) < 0.3:
            self.position_offset += 0.2*delta_position
            self.heading_offset += 0.3*delta_heading

    def get_position(self):
        pos = self.position_odometry + self.position_offset
        heading = self.heading_odometry + self.heading_offset
        return pos.tolist() + [periodic_error(heading)]


def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    pose = PositionEstimator()
    o = node.recv('/odometry_raw')
    pose.update_odometry(o)
    pose.reset([0.5, 0.5], 0)

    while True:
        try:
            o = node.recv('/odometry_raw', timeout=0)
            pose.update_odometry(o)
            node.publish('/position', pose.get_position())
        except queue.Empty:
            pass
        try:
            l = node.recv('/lidar/position', timeout=0)
            pose.update_lidar(l)
            node.publish('/position', pose.get_position())
        except queue.Empty:
            pass
        time.sleep(0.01)

if __name__ == '__main__':
    main()
