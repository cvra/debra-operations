import zmqmsgbus
import time
from arm_trajectories import scara
from collections import OrderedDict
import sys
import homing_handler
from math import pi

class Arm:
    def __init__(self, upper_arm=0.14, forearm=0.052, z_meter_per_rad=0.005/2/pi):
        self.upper_arm = upper_arm
        self.forearm = forearm
        self.z_meter_per_rad = z_meter_per_rad
        self.joints = OrderedDict([('z', 0),
                                   ('shoulder', 0),
                                   ('elbow', 0),
                                   ('wrist', 0)])
        self.joint_limits = OrderedDict([('z', (0, 0.21)),
                                   ('shoulder', (-2.1, 2.1)),
                                   ('elbow', (-2.2, 2.2)),
                                   ('wrist', (-float('inf'), float('inf')))])
        self.zeros = OrderedDict([('z', 0),
                                   ('shoulder', 0),
                                   ('elbow', 0),
                                   ('wrist', 0)])

    def set_zeros(self, zeros):
        for actuator in zeros:
            self.zeros[actuator] = zeros[actuator]

    def get_hand_position(self):
        return scara.forward_kinematics([self.joints['shoulder'],
                                         self.joints['elbow'],
                                         self.joints['z']],
                                        self.upper_arm,
                                        self.forearm,
                                        self.z_meter_per_rad)

    def move_hand(self, x, y, z):
        limits = [self.joint_limits['shoulder'], self.joint_limits['elbow'], self.joint_limits['z']]
        shoulder, elbow, z_actuator = scara.inverse_kinematics([x, y, z],
                                                               self.upper_arm,
                                                               self.forearm,
                                                               self.z_meter_per_rad,
                                                               limits)
        self.joints['shoulder'] = shoulder
        self.joints['elbow'] = elbow
        self.joints['z'] = z_actuator

    def get_actuators(self):
        res = self.joints.copy()
        for actuator in res:
            res[actuator] = res[actuator] + self.zeros[actuator]
        return res


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

    indexer = homing_handler.Indexer()
    indexer.add(['left-shoulder', 'left-elbow', 'left-wrist'])
    l.set_zeros(indexer.start())
    l.set_zeros({'z': -0.21/(0.005/2/pi)})

    while True:
        hand = node.recv('/left-arm/hand/setpoint')
        l.move_hand(*[float(v) for v in hand])
        actuator_position(node, 'left', l.get_actuators())


if __name__ == '__main__':
    main()
