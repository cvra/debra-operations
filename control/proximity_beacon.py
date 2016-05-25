import zmq
import zmqmsgbus
import unittest
import math
import time
import queue
import copy
from threading import Lock

HEADING_OFFSET = math.pi/2
REFLECTOR_DIAMETER = 79e-3
PUBLSIH_FREQUENCY = 5
MAX_NB_SIGNALS = 6

def relative_heading_and_distance(start, length):
    if start < 0 or start > 2*math.pi or length <= 0 or length > 2*math.pi:
        return
    global HEADING_OFFSET
    global REFLECTOR_DIAMETER
    dist = REFLECTOR_DIAMETER/2 / math.tan(length/2)
    head = start + length/2 + HEADING_OFFSET
    head = math.fmod(head, 2*math.pi)
    return [head, dist]

def relative_to_global_position(x, y, theta, dist, angle):
    x += dist * math.cos(theta + angle)
    y += dist * math.sin(theta + angle)
    return [x, y]

def proximity_beacon_msg(queue, position, msg):
    start, length = msg
    angle, dist = relative_heading_and_distance(start, length)
    x, y, theta = position.get()
    pos = relative_to_global_position(x, y, theta, dist, angle)
    queue.put(pos)

class PositionObject:
    def __init__(self, init=[0,0,0]):
        self.lock = Lock()
        self.position = init

    def get(self):
        with self.lock:
            return copy.copy(self.position)

    def set(self, position):
        with self.lock:
            self.position = copy.copy(position)

class SignalQueue:
    def __init__(self):
        self.q = queue.Queue(MAX_NB_SIGNALS)
        self.lock = Lock()

    def put(self, pos):
        with self.lock:
            try:
                self.q.put_nowait(pos)
            except queue.Full:
                pass # drop data, should never happen

    def get(self):
        with self.lock:
            l = list()
            try:
                while True:
                    l.append(self.q.get_nowait())
            except queue.Empty:
                return l

def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    position = PositionObject()
    queue = SignalQueue()
    handler = lambda topic, msg: position.set(msg)
    node.register_message_handler('/position', handler)
    handler = lambda topic, msg: proximity_beacon_msg(queue, position, msg)
    node.register_message_handler('/proximity_beacon', handler)

    while True:
        time.sleep(1/PUBLSIH_FREQUENCY)
        node.publish('/obstacles', queue.get())

if __name__ == '__main__':
    main()

class ProximityBeaconTestCase(unittest.TestCase):

    def test_distance(self):
        global HEADING_OFFSET
        global REFLECTOR_DIAMETER
        expect_dist = 2
        length = 2*math.atan(REFLECTOR_DIAMETER/2 / expect_dist)
        head, dist = relative_heading_and_distance(0, length)
        self.assertAlmostEqual(dist, expect_dist)
        self.assertAlmostEqual(head, 0 + length/2 + HEADING_OFFSET)

    def test_heading_is_always_smaller_than_two_pi(self):
        head, dist = relative_heading_and_distance(2*math.pi - 0.0001, 0.1)
        self.assertTrue(head < 2*math.pi)
        self.assertTrue(head >= 0)

    def test_relative_to_global_position(self):
        x, y = relative_to_global_position(0,0,0,1,math.pi/2)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 1)

        x, y = relative_to_global_position(1,2,0,1,0)
        self.assertAlmostEqual(x, 2)
        self.assertAlmostEqual(y, 2)

