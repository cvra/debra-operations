import zmqmsgbus
import time
from arm_trajectories import scara
from collections import OrderedDict
import sys
import queue
import homing_handler
from math import pi
import math


class Arm:
    """
    Hand: theta=0 in the direction hand center to TCP
    2 5 1
     \ /
      o
     / \
    3   4
    """

    def __init__(self,
                 upper_arm=0.14,
                 forearm=0.052,
                 z_meter_per_rad=0.005/2/pi,
                 tool_1_angle=pi,
                 tool_distance=0.03,
                 tool_5_distance=0.07,
                 tool_5_z_offset=0.03):
        self.upper_arm = upper_arm
        self.forearm = forearm
        self.z_meter_per_rad = z_meter_per_rad
        self.tool_1_angle = tool_1_angle
        self.tool_distance = tool_distance
        self.tool_5_distance = tool_5_distance
        self.tool_5_z_offset = tool_5_z_offset
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

    def get_hand_orientation(self):
        return self.joints['shoulder'] + self.joints['elbow'] + self.joints['wrist']

    def move_hand(self, x, y, z, theta):
        limits = [self.joint_limits['shoulder'], self.joint_limits['elbow'], self.joint_limits['z']]
        shoulder, elbow, z_actuator = scara.inverse_kinematics([x, y, z],
                                                               self.upper_arm,
                                                               self.forearm,
                                                               self.z_meter_per_rad,
                                                               limits)
        self.joints['shoulder'] = shoulder
        self.joints['elbow'] = elbow
        self.joints['z'] = z_actuator
        self.joints['wrist'] = theta - shoulder - elbow

    def move_tcp(self, tool, x, y, z, theta):
        tcp_offset = self.get_tcp_offset(tool, theta)
        self.move_hand(x - tcp_offset[0],
                       y - tcp_offset[1],
                       z - tcp_offset[2],
                       theta - tcp_offset[3])

    def get_tcp_offset(self, tool, theta):
        """ Offset from TCP to center of hand in the arm reference frame.
        """
        if tool not in [1, 2, 3, 4, 5]:
            raise ValueError

        tool_angles = [0, pi/2, pi, 3/2*pi, pi/4]
        theta_hand = self.tool_1_angle + tool_angles[tool-1]
        if tool == 5:
            x = self.tool_5_distance * math.cos(theta)
            y = self.tool_5_distance * math.sin(theta)
            z = self.tool_5_z_offset
        else:
            x = self.tool_distance * math.cos(theta)
            y = self.tool_distance * math.sin(theta)
            z = 0

        return x, y, z, theta_hand

    def get_actuators(self):
        res = self.joints.copy()
        for actuator in res:
            res[actuator] = res[actuator] + self.zeros[actuator]
        return res


def actuator_position(node, arm, joints):
    node.call('/actuator/position', [arm + '-shoulder', -joints['shoulder']])
    node.call('/actuator/position', [arm + '-elbow', -joints['elbow']])
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
        try:
            setpoint = node.recv('/left-arm/setpoint')
            if setpoint[0] == 0:
                l.move_hand(*[float(v) for v in setpoint[1:]])
            else:
                l.move_tcp(int(setpoint[0]), *[float(v) for v in setpoint[1:]])
            actuator_position(node, 'left', l.get_actuators())
        except queue.Empty:
            pass
        except ValueError as e:
            print(e)
        time.sleep(0.01)


if __name__ == '__main__':
    main()
