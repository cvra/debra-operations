import unittest
import position
from math import cos, sin, pi
import numpy as np


class TestPosition(unittest.TestCase):

    def test_map_odometry_to_table(self):
        odometry_pos = [1, 2, 3]
        offset = [0.1, 0.2, 0.3]
        table_pos = position.map_odometry_to_table(odometry_pos, offset)
        self.assertAlmostEqual(cos(0.3) * 1 - sin(0.3) * 2 + 0.1, table_pos[0])
        self.assertAlmostEqual(sin(0.3) * 1 + cos(0.3) * 2 + 0.2, table_pos[1])
        self.assertAlmostEqual(3 + 0.3, table_pos[2])

    def test_kalman_state_propagation(self):
        P = np.matrix([[1, 2], [2, 4]])
        Phi = np.matrix([[3, 2], [1, 0]])
        Q =  np.matrix([[0.1, 0], [0, 0.2]])

        np.testing.assert_almost_equal(Phi * P * Phi.T + Q,
                                       position.kalman_state_propagation(P, Phi, Q))

    def test_kalman_measurement_update(self):
        x = np.matrix([[1], [2]])
        z = np.matrix([1])
        P = np.matrix([[1, 2], [2, 4]])
        h_of_x = np.matrix([0.9])
        z_minus_h_of_x = z - h_of_x
        H = np.matrix([1, 2])
        R = np.matrix([0.1])

        S = H * P * H.T + R
        K = P * H.T * np.linalg.inv(S)
        P_kp1 = (np.eye(len(x)) - K * H) * P
        x_kp1 = x + K * (z - h_of_x)

        x_test, P_test = position.kalman_measurement_update(x, P, z_minus_h_of_x, H, R)
        np.testing.assert_almost_equal(x_kp1, x_test)
        np.testing.assert_almost_equal(P_kp1, P_test)

    def test_compute_odometry_offset(self):
        table_pos = [1, 2, 3]
        odometry_pos = [4, 5, 6]
        offset = position.compute_odometry_offset(table_pos, odometry_pos)
        np.testing.assert_almost_equal(table_pos, position.map_odometry_to_table(odometry_pos, offset))

    def test_position_estimator_reset_and_get_position(self):
        p = position.PositionEstimator()
        p.update_odometry([3, 4, 5])
        p.reset([1, 2, 3])
        np.testing.assert_almost_equal([1, 2, 3], p.get_position())
