import unittest
from cst_vel_homing import center_between_periodic_angles, normalize_angle
from math import pi


class TestCstVelHoming(unittest.TestCase):

    def test_normalize_angle(self):
        self.assertAlmostEqual(2, normalize_angle(2))
        self.assertAlmostEqual(2, normalize_angle(2+2*pi))
        self.assertAlmostEqual(2, normalize_angle(2+4*pi))
        self.assertAlmostEqual(2, normalize_angle(2-2*pi))
        self.assertAlmostEqual(-3, normalize_angle(-3-2*pi))
        self.assertAlmostEqual(4-2*pi, normalize_angle(4))

    def test_center_between_periodic_angles(self):
        self.assertAlmostEqual(0, center_between_periodic_angles(-1, 1))
        self.assertAlmostEqual(pi, center_between_periodic_angles(-2, 2))
        self.assertAlmostEqual(pi-0.2, center_between_periodic_angles(-2.2, 1.8))
        self.assertAlmostEqual(pi+0.2, center_between_periodic_angles(-1.8, 2.2))

