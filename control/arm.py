import zmqmsgbus
import time
from arm_trajectories import scara
from collections import OrderedDict
import sys
import queue
import homing_handler
from math import pi
import math
import yaml


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
                 tool_distance=0.03,
                 tool_5_distance=0.07,
                 tool_5_z_offset=0.03):
        self.upper_arm = upper_arm
        self.forearm = forearm
        self.z_meter_per_rad = z_meter_per_rad
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

        tool_angles = [-pi/4, pi/4, 3/4*pi, -3/4*pi, 0]
        theta_hand = tool_angles[tool-1]
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


def actuator_position(node, arm, joints, offsets):
    node.call('/actuator/position',
              [arm + '-shoulder', -1*(joints['shoulder'] + offsets[arm + '-shoulder'])])
    node.call('/actuator/position',
              [arm + '-elbow', -1*(joints['elbow'] + offsets[arm + '-elbow'])])
    node.call('/actuator/position',
              [arm + '-wrist', joints['wrist'] + offsets[arm + '-wrist']])
    node.call('/actuator/position',
              [arm + '-z', joints['z'] + offsets[arm + '-z']])


def map_body_to_arm_frame(x, y, z, theta, arm):
    SHOULDER_POSITION = 0.1
    if arm == 'left':
        return y - SHOULDER_POSITION, -x, z, theta - pi/2
    if arm == 'right':
        return -y - SHOULDER_POSITION, x, z, theta + pi/2


def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    time.sleep(1)

    offsets = yaml.load(open(sys.argv[1]))
    left_arm_config = yaml.load(open("config/config-left-arm.yaml"))
    right_arm_config = yaml.load(open("config/config-right-arm.yaml"))

    l = Arm()
    r = Arm()

    endstopper = homing_handler.Endstopper()
    endstopper.add(['left-z'], 50, left_arm_config['actuator']['left-z']['control']['torque_limit'])
    endstopper.add(['right-z'], 50, right_arm_config['actuator']['right-z']['control']['torque_limit'])
    l.set_zeros({a[len('left-'):]: z for a, z in endstopper.start().items() if 'left-' in a})
    r.set_zeros({a[len('right-'):]: z for a, z in endstopper.start().items() if 'right-' in a})

    indexer = homing_handler.Indexer()
    indexer.add(['left-shoulder', 'left-elbow', 'left-wrist'])
    indexer.add(['right-shoulder', 'right-elbow', 'right-wrist'])
    l.set_zeros({a[len('left-'):]: z for a, z in indexer.start().items() if 'left-' in a})
    r.set_zeros({a[len('right-'):]: z for a, z in indexer.start().items() if 'right-' in a})

    l.move_hand(0.192, 0, 0.18, 0)
    r.move_hand(0.192, 0, 0.18, 0)

    actuator_position(node, 'left', l.get_actuators(), offsets)
    actuator_position(node, 'right', r.get_actuators(), offsets)

    while True:
        try:
            setpoint = node.recv('/left-arm/setpoint', timeout=0)
            setpoint = [int(setpoint[0])] + list(map_body_to_arm_frame(*[float(v) for v in setpoint[1:]], arm='left'))
            if setpoint[0] == 0:
                l.move_hand(*setpoint[1:])
            else:
                l.move_tcp(*setpoint)
            actuator_position(node, 'left', l.get_actuators(), offsets)
        except queue.Empty:
            pass
        except ValueError as e:
            print(e)
        try:
            setpoint = node.recv('/right-arm/setpoint', timeout=0)
            setpoint = [int(setpoint[0])] + list(map_body_to_arm_frame(*[float(v) for v in setpoint[1:]], arm='right'))
            if setpoint[0] == 0:
                r.move_hand(*setpoint[1:])
            else:
                r.move_tcp(*setpoint)
            actuator_position(node, 'right', r.get_actuators(), offsets)
        except queue.Empty:
            pass
        except ValueError as e:
            print(e)
        time.sleep(0.01)


if __name__ == '__main__':
    main()
