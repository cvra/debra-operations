import unittest
from unittest.mock import *
from homing_handler import Indexer

class HomingHandlerTestCase(unittest.TestCase):
    @patch('homing_handler.zmqmsgbus.Node')
    def test_can_create_indexer(self, open_package_mock):
        indexer = Indexer()

    @patch('homing_handler.zmqmsgbus.Node')
    def test_can_create_indexer_actuator(self, open_package_mock):
        indexer = Indexer()
        actuator = indexer.Actuator('test')

    def test_unwind(self):
        from math import pi
        value = Indexer.unwind(2*pi - 0.1)
        self.assertAlmostEqual(-0.1, value)
        value = Indexer.unwind(0.1)
        self.assertAlmostEqual(0.1, value)
        value = Indexer.unwind(-0.1)
        self.assertAlmostEqual(-0.1, value)
        value = Indexer.unwind(0.1 - 2*pi)
        self.assertAlmostEqual(0.1, value)


class IndexCallbackTestCase(unittest.TestCase):
    @patch('homing_handler.zmqmsgbus.Node')
    def test_basic_case(self, open_package_mock):
        indexer = Indexer()
        actuator = indexer.Actuator('test')

        actuator.pos_setpoint = 2

        indexer.index_callback(actuator, 1)
        self.assertEqual(1.3, actuator.pos_setpoint)
        indexer.index_callback(actuator, 1.2)

        self.assertEqual(1.1, actuator.index_pos)
        self.assertTrue(actuator.stop)

    @patch('homing_handler.zmqmsgbus.Node')
    def test_case_with_zero(self, open_package_mock):
        indexer = Indexer()
        actuator = indexer.Actuator('test')

        actuator.pos_setpoint = 2
        indexer.index_callback(actuator, 0)
        self.assertEqual(0.3, actuator.pos_setpoint)
        indexer.index_callback(actuator, 0.2)

        self.assertEqual(0.1, actuator.index_pos)
        self.assertTrue(actuator.stop)

    @patch('homing_handler.zmqmsgbus.Node')
    def test_ignores_small_changes(self, open_package_mock):
        indexer = Indexer()
        actuator = indexer.Actuator('test')
        indexer.index_callback(actuator, 0)
        indexer.index_callback(actuator, 0.05)

        self.assertFalse(actuator.stop)
        self.assertEqual(0, actuator.index_pos)


class PositionCallbackTestCase(unittest.TestCase):
    @patch('homing_handler.zmqmsgbus.Node')
    def test_basic_case(self, open_package_mock):
        pass

class MessageCallbackeTestCase(unittest.TestCase):
    @patch('homing_handler.zmqmsgbus.Node')
    def test_index_redirection(self, open_package_mock):
        indexer = Indexer()
        indexer.add(['test'])

        indexer.message_cb('/actuator/test/index', 42)

        self.assertEqual(42, indexer.actuators[0].index_pos)

    @patch('homing_handler.zmqmsgbus.Node')
    def test_index_redirection(self, open_package_mock):
        indexer = Indexer()
        indexer.add(['test'])

        indexer.message_cb('/actuator/test/position', 1)
        self.assertEqual(1.3, indexer.actuators[0].pos_setpoint)
        indexer.message_cb('/actuator/test/position', 1.3)
        self.assertAlmostEqual(0.7, indexer.actuators[0].pos_setpoint)
