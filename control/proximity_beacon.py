import zmq
import zmqmsgbus
import unittest
import math
import time

HEADING_OFFSET = math.pi/2
REFLECTOR_DIAMETER = 79e-3

def calculate_heading_and_distance(start, length):
    if start < 0 or start > 2*math.pi or length <= 0 or length > 2*math.pi:
        return
    global HEADING_OFFSET
    global REFLECTOR_DIAMETER
    dist = REFLECTOR_DIAMETER/2 / math.tan(length/2)
    head = start + length/2 + HEADING_OFFSET
    head = math.fmod(head, 2*math.pi)
    return [head, dist]

def proximity_beacon_msg(node, msg):
    start, length = msg
    head, dist = calculate_heading_and_distance(start, length)
    msg = {'heading': head, 'distance': dist}
    node.publish('/bacon', msg) # todo: choose msg name

def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    handler = lambda topic, msg: proximity_beacon_msg(node, msg)
    node.register_message_handler('/proximity_beacon', handler)

    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()

class ProximityBeaconTestCase(unittest.TestCase):
    def test_distance_calculations(self):
        global HEADING_OFFSET
        global REFLECTOR_DIAMETER
        expect_dist = 2
        length = 2*math.atan(REFLECTOR_DIAMETER/2 / expect_dist)
        head, dist = calculate_heading_and_distance(0, length)
        self.assertAlmostEqual(dist, expect_dist)
        self.assertAlmostEqual(head, 0 + length/2 + HEADING_OFFSET)
    def test_heading_is_always_smaller_than_two_pi(self):
        head, dist = calculate_heading_and_distance(2*math.pi - 0.0001, 0.1)
        self.assertTrue(head < 2*math.pi)
        self.assertTrue(head >= 0)

