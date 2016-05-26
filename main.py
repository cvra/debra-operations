import zmqmsgbus
import queue
import time
import logging
from math import pi


table_length = 3
table_width = 2
start_position = {'violet': [0.85, 0.129, -pi/2],
                  'green': [0.85, table_length - 0.129, +pi/2]}


enemy_color = {'violet': 'green', 'green': 'violet'}
button_and_led_color = {'violet': 'yellow', 'green': 'green'}


def zero_torque(node):
    node.call("/actuator/torque", ["left-wheel", 0])
    node.call("/actuator/torque", ["right-wheel", 0])
    node.call("/actuator/torque", ["left-z", 0])
    node.call("/actuator/torque", ["left-shoulder", 0])
    node.call("/actuator/torque", ["left-elbow", 0])
    node.call("/actuator/torque", ["left-wrist", 0])
    node.call("/actuator/torque", ["right-z", 0])
    node.call("/actuator/torque", ["right-shoulder", 0])
    node.call("/actuator/torque", ["right-elbow", 0])
    node.call("/actuator/torque", ["right-wrist", 0])


def get_start_color(node):
    green = False
    yellow = False
    node.call("/actuator/led_set", ["green_1", False])
    node.call("/actuator/led_set", ["green_2", False])
    node.call("/actuator/led_set", ["yellow_1", False])
    node.call("/actuator/led_set", ["yellow_2", False])

    while True:
        try:
            green = node.recv('/interface-panel/green-pressed', timeout=0)
        except queue.Empty:
            pass
        try:
            yellow = node.recv('/interface-panel/yellow-pressed', timeout=0)
        except queue.Empty:
            pass
        if green:
            node.call("/actuator/led_set", ["green_1", True])
            node.call("/actuator/led_set", ["green_2", True])
            return "green"
        if yellow:
            node.call("/actuator/led_set", ["yellow_1", True])
            node.call("/actuator/led_set", ["yellow_2", True])
            return "violet"


def wait_for_start(node):
    node.call("/actuator/led_set", ["ready", False])
    while True:
        start = node.recv('/interface-panel/start')
        if start is False:
            break
    node.call("/actuator/led_set", ["ready", True])

    while True:
        start = node.recv('/interface-panel/start')
        if start is True:
            break

    node.call("/actuator/led_set", ["ready", False])


def init_sequence(node):
    team_color = get_start_color(node)
    logging.info(team_color)

    logging.debug('resetting to position: {}'.format(start_position[team_color]))
    node.call("/position/reset", start_position[team_color])
    return team_color


def main():
    logging.basicConfig(level=logging.DEBUG)

    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    time.sleep(1.1)

    zero_torque(node)

    team_color = None
    while team_color is None:
        team_color = init_sequence(node)

    wait_for_start(node)
    logging.info("start!")


if __name__ == '__main__':
    main()
