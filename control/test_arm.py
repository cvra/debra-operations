import unittest
import arm


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
        a.move_hand(5, 3, 0.1)
        pos = a.get_hand_position()
        self.assertAlmostEqual(pos[0], 5)
        self.assertAlmostEqual(pos[1], 3)
        self.assertAlmostEqual(pos[2], 0.1)

    def test_zeros(self):
        a = arm.Arm()
        a.set_zeros({'z': 1, 'shoulder': 2, 'elbow': 3, 'wrist': 4})
        self.assertEqual(list(a.get_actuators().values()), [1, 2, 3, 4])
