import zmqmsgbus
import time
from arm_trajectories import scara
from collections import OrderedDict
import sys


class Arm:
    def __init__(self, upper_arm=2, forearm=1):
        self.upper_arm = upper_arm
        self.forearm = forearm
        self.joints = OrderedDict([('z', 0),
                                   ('shoulder', 0),
                                   ('elbow', 0),
                                   ('wrist', 0)])
        self.joint_limits = OrderedDict([('z', (0, 250)),
                                   ('shoulder', (-2.1, 2.1)),
                                   ('elbow', (-2.2, 2.2)),
                                   ('wrist', (-float('inf'), float('inf')))])

    def get_hand_position(self):
        xy = scara.forward_kinematics([self.joints['shoulder'], self.joints['elbow']], self.upper_arm, self.forearm)
        return xy + (self.joints['z'], )

    def move_hand(self, x, y, z):
        limits = [self.joint_limits['shoulder'], self.joint_limits['elbow']]
        shoulder, elbow = scara.inverse_kinematics([x, y], self.upper_arm, self.forearm, limits)
        self.joints['shoulder'] = shoulder
        self.joints['elbow'] = elbow
        self.joints['z'] = z

    def get_actuators(self):
        return self.joints


def actuator_position(node, arm, joints):
    node.call('/actuator/position', [arm + '-shoulder', joints['shoulder']])
    node.call('/actuator/position', [arm + '-elbow', joints['elbow']])
    node.call('/actuator/position', [arm + '-wrist', joints['wrist']])
    node.call('/actuator/position', [arm + '-z', joints['z']])


def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    time.sleep(1)

    l = Arm()

    l.move_hand(float(sys.argv[1]), float(sys.argv[2]), 200)
    actuator_position(node, 'left', l.get_actuators())


if __name__ == '__main__':
    main()
