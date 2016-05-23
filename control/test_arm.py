import unittest
import arm
from math import pi
import math

class TestArm(unittest.TestCase):

    def test_init(self):
        a = arm.Arm(10, 1)
        self.assertEqual(list(a.get_actuators().values()), [0, 0, 0, 0])

    def test_arm_length(self):
        a = arm.Arm(10, 1)
        pos = a.get_hand_position()
        self.assertEqual(pos[0], 11)
        self.assertEqual(pos[1], 0)

    def test_inverse_kinematics(self):
        a = arm.Arm(5, 4, 1)
        a.move_hand(5, 3, 0.1, 0)
        pos = a.get_hand_position()
        self.assertAlmostEqual(pos[0], 5)
        self.assertAlmostEqual(pos[1], 3)
        self.assertAlmostEqual(pos[2], 0.1)

    def test_move_hand(self):
        a = arm.Arm(5, 4, 1)
        a.move_hand(5, 4, 0.1, 0)
        joint = a.get_actuators()
        self.assertAlmostEqual(joint['shoulder'], 0)
        self.assertAlmostEqual(joint['elbow'], pi/2)
        self.assertAlmostEqual(joint['z'], 0.1)
        self.assertAlmostEqual(joint['wrist'], -pi/2)

    def test_zeros(self):
        a = arm.Arm()
        a.set_zeros({'z': 1, 'shoulder': 2, 'elbow': 3, 'wrist': 4})
        self.assertEqual(list(a.get_actuators().values()), [1, 2, 3, 4])

    def test_get_tcp_1_offset(self):
        a = arm.Arm(1, 1, 1, pi/2, 1, 2, 1)
        theta = pi/2
        x, y, z, theta_hand = a.get_tcp_offset(tool=1, theta=theta)
        self.assertAlmostEqual(pi/2, theta_hand)
        self.assertAlmostEqual(1 * math.cos(theta), x)
        self.assertAlmostEqual(1 * math.sin(theta), y)
        self.assertAlmostEqual(0, z)

    def test_get_tcp_2_offset(self):
        a = arm.Arm(1, 1, 1, pi/2, 1, 2, 1)
        theta = pi/2
        x, y, z, theta_hand = a.get_tcp_offset(tool=2, theta=theta)
        self.assertAlmostEqual(pi/2 + pi/2, theta_hand)
        self.assertAlmostEqual(1 * math.cos(theta), x)
        self.assertAlmostEqual(1 * math.sin(theta), y)
        self.assertAlmostEqual(0, z)

    def test_get_tcp_4_offset(self):
        a = arm.Arm(1, 1, 1, pi/2, 1, 2, 1)
        theta = pi/2
        x, y, z, theta_hand = a.get_tcp_offset(tool=4, theta=theta)
        self.assertAlmostEqual(pi/2 + 3/2*pi, theta_hand)
        self.assertAlmostEqual(1 * math.cos(theta), x)
        self.assertAlmostEqual(1 * math.sin(theta), y)
        self.assertAlmostEqual(0, z)

    def test_inexistent_tcp(self):
        a = arm.Arm()
        with self.assertRaises(ValueError):
            a.move_tcp(42, 0, 0, 0, 0)

    def test_get_tcp_5_offset(self):
        a = arm.Arm(1, 1, 1, pi/2, 1, 2, 1)
        theta = pi/2
        x, y, z, theta_hand = a.get_tcp_offset(tool=5, theta=theta)
        self.assertAlmostEqual(pi/2 + pi/4, theta_hand)
        self.assertAlmostEqual(2 * math.cos(theta), x)
        self.assertAlmostEqual(2 * math.sin(theta), y)
        self.assertAlmostEqual(1, z)

    def test_move_tcp(self):
        a = arm.Arm(5, 4, 1)
        a.move_tcp(tool=1, x=5, y=3, z=1, theta=0)
        hand = a.get_hand_position()
        hand_orientation = a.get_hand_orientation()
        tcp_offset = a.get_tcp_offset(tool=1, theta=0)
        self.assertAlmostEqual(5, hand[0] + tcp_offset[0])
        self.assertAlmostEqual(3, hand[1] + tcp_offset[1])
        self.assertAlmostEqual(1, hand[2] + tcp_offset[2])
        self.assertAlmostEqual(0, hand_orientation + tcp_offset[3])
