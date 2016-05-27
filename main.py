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
    node.call("/actuator/torque", ["left-elbow", 0])
    node.call("/actuator/torque", ["left-wrist", 0])
    node.call("/actuator/torque", ["right-z", 0])
    node.call("/actuator/torque", ["right-elbow", 0])
    node.call("/actuator/torque", ["right-wrist", 0])
    node.call("/actuator/torque", ["left-shoulder", 0])
    node.call("/actuator/torque", ["right-shoulder", 0])


def zero_velocity(node):
    node.call("/actuator/velocity", ["left-wheel", 0])
    node.call("/actuator/velocity", ["right-wheel", 0])
    node.call("/actuator/velocity", ["left-z", 0])
    node.call("/actuator/velocity", ["left-elbow", 0])
    node.call("/actuator/velocity", ["left-wrist", 0])
    node.call("/actuator/velocity", ["right-z", 0])
    node.call("/actuator/velocity", ["right-elbow", 0])
    node.call("/actuator/velocity", ["right-wrist", 0])
    node.call("/actuator/velocity", ["left-shoulder", 0])
    node.call("/actuator/velocity", ["right-shoulder", 0])


def safe_arm_position(node):
    node.publish('/left-arm/setpoint', [5, 0.14, 0.0, 0.185, -pi/2])
    node.publish('/right-arm/setpoint', [5, 0.14, 0.0, 0.185, pi/2])
    time.sleep(1)


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
            team_color = "green"
            break
        if yellow:
            team_color = "violet"
            break

    node.call("/actuator/led_set", [button_and_led_color[team_color] + '_1', True])
    while node.recv('/interface-panel/{}-pressed'.format(button_and_led_color[team_color])) is True:
        pass # wait for release
    return team_color


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

    while node.recv('/interface-panel/{}-pressed'.format(button_and_led_color[team_color])) is False:
        pass

    zero_velocity(node)
    node.call("/actuator/led_set", [button_and_led_color[team_color] + '_2', True])
    node.call("/arm/run_zero_homing", None)
    safe_arm_position(node)

    return team_color


def main():
    logging.basicConfig(level=logging.DEBUG)

    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    time.sleep(1.1)

    logging.info("ready")

    zero_torque(node)

    team_color = None
    while team_color is None:
        team_color = init_sequence(node)

    node.publish('/waypoint', [0.95, 0.17, 1.57079])

    wait_for_start(node)
    logging.info("start!")

    node.publish('/waypoint', [0.6, 0.65, pi/2])
    time.sleep(5)

    # right-pump-5, 12
    # right-pump-1, 12
    # right-pump-2, 12
    # right-pump-3, 12
    # right-pump-4, -12

    # above cylinder
    node.publish('/right-arm/table-setpoint', [5, 0.8, 0.9, 0.24, pi/2])
    time.sleep(1)
    node.publish('/right-arm/table-setpoint', [5, 0.9, 0.6, 0.24, pi/2])
    time.sleep(2)

    # descend
    node.publish('/right-arm/table-setpoint', [5, 0.9, 0.635, 0.118, pi/2])
    time.sleep(2)

    node.call("/actuator/voltage", ['right-pump-5', 15])
    time.sleep(2)
    # grab
    node.publish('/right-arm/table-setpoint', [5, 0.9, 0.68, 0.118, pi/2])
    time.sleep(2)

    # ascend
    node.publish('/right-arm/table-setpoint', [5, 0.9, 0.68, 0.18, pi/2])
    time.sleep(2)

    # hand over
    # TODO activate other pump
    node.publish('/left-arm/setpoint', [5, 0.14, -0.01, 0.185, -1.6])
    node.publish('/right-arm/setpoint', [5, 0.14, 0.01, 0.185, 1.6])
    time.sleep(2)

    node.call("/actuator/voltage", ['right-pump-5', 0])


    node.publish('/right-arm/table-setpoint', [0, 0.9, 0.65, 0.1, pi])
    time.sleep(2)
    node.call("/actuator/voltage", ['right-pump-1', -12])
    node.call("/actuator/voltage", ['right-pump-2', 12])
    node.call("/actuator/voltage", ['right-pump-3', 12])
    node.call("/actuator/voltage", ['right-pump-4', -12])
    time.sleep(2)
    node.publish('/right-arm/table-setpoint', [0, 0.9, 0.65, 0.062, pi])
    time.sleep(2)
    node.publish('/right-arm/table-setpoint', [0, 0.9, 0.65, 0.15, pi])
    time.sleep(2)

    node.publish('/waypoint', [0.57, 1.2, pi/2])
    time.sleep(5)
    node.publish('/right-arm/table-setpoint', [0, 0.88, 1.2, 0.15, pi])
    time.sleep(2)
    # place blocks in center
    node.publish('/right-arm/table-setpoint', [0, 0.88, 1.2, 0.07, pi])
    time.sleep(2)
    node.call("/actuator/voltage", ['right-pump-1', 0])
    node.call("/actuator/voltage", ['right-pump-2', 0])
    node.call("/actuator/voltage", ['right-pump-3', 0])
    node.call("/actuator/voltage", ['right-pump-4', 0])
    time.sleep(2)

    node.publish('/right-arm/table-setpoint', [0, 0.88, 1.2, 0.15, pi])


if __name__ == '__main__':
    main()
