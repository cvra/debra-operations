import unittest
from unittest.mock import Mock
import main
from math import pi


class TestMapping(unittest.TestCase):

    def test_flip_table_xytheta(self):
        self.assertEqual([1, 3-2, -3], main.flip_table_xytheta([1, 2, 3]))

    def test_move_arm_in_table_frame(self):
        node = Mock()
        pos = [1, 1, 2, 3, pi/3]
        main.move_arm_in_table_frame(node, 'green', 'left', pos)
        node.publish.assert_called_once_with('/right-arm/table-setpoint', [2, 1, main.table_length - 2, 3, -pi/3])

    def test_move_arm_in_body_frame(self):
        node = Mock()
        pos = [1, 1, 2, 3, pi/3]
        main.move_arm_in_body_frame(node, 'green', 'left', pos)
        node.publish.assert_called_once_with('/right-arm/setpoint', [2, 1, -2, 3, -pi/3])

    def test_set_waypoint(self):
        node = Mock()
        pos = [1, 2, pi/3]
        main.set_waypoint(node, 'green', pos)
        node.publish.assert_called_once_with('/waypoint', [1, main.table_length - 2, -pi/3])

    def test_set_pump(self):
        node = Mock()
        main.set_pump(node, 'green', 'left', 1, 12)
        node.call.assert_called_once_with('/actuator/voltage', ['right-pump-2', -12])