import zmqmsgbus
import queue
import time


class PositionEstimator():

    def __init__(self):
        self.position_odometry = [0, 0]
        self.heading_odometry = 0
        self.position_offset = [0, 0]
        self.heading_offset = 0

    def update_odometry(self, odometry):
        self.position_odometry = odometry[0:2]
        self.heading_odometry = odometry[2]

    def update_lidar(self, lidar):
        pass

    def get_position(self):
        return self.position_odometry + [self.heading_odometry]


def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    pose = PositionEstimator()

    while True:
        try:
            o = node.recv('/odometry_raw', timeout=0)
            pose.update_odometry(o)
            node.publish('/position', pose.get_position())
        except queue.Empty:
            pass
        try:
            o = node.recv('/lidar/position', timeout=0)
            pose.update_odometry(o)
            node.publish('/position', pose.get_position())
        except queue.Empty:
            pass
        time.sleep(0.01)

if __name__ == '__main__':
    main()
