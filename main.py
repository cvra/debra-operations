import zmqmsgbus
import queue
import time
import logging
from math import pi
import numpy as np
import threading


actuator_lock = threading.RLock()

table_length = 3
table_width = 2
# debra width 205mm
start_position = {'violet': [0.4975, 0.129, -pi/2],
                  'green': [0.4975, table_length - 0.129, +pi/2]}


enemy_color = {'violet': 'green', 'green': 'violet'}
button_and_led_color = {'violet': 'yellow', 'green': 'green'}

flip_left_right = {'left': 'right', 'right': 'left'}
flip_hand_tool = {0: 0, 1: 2, 2: 1, 3: 4, 4: 3, 5: 5}


def flip_table_xytheta(xytheta):
    xytheta[1] = table_length - xytheta[1] # flip y
    xytheta[2] = - xytheta[2] # flip theta
    return xytheta


def zero_torque(node):
    with actuator_lock:
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
    with actuator_lock:
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


def move_arm_in_table_frame(node, team_color, arm, pos):
    if team_color is 'green':
        arm = flip_left_right[arm]
        pos[1], pos[2], pos[4] = flip_table_xytheta([pos[1], pos[2], pos[4]])
        pos[0] = flip_hand_tool[pos[0]]
    with actuator_lock:
        node.publish('/{}-arm/table-setpoint'.format(arm), pos)


def move_arm_in_body_frame(node, team_color, arm, pos):
    if team_color is 'green':
        arm = flip_left_right[arm]
        pos[2] = - pos[2] # flip y
        pos[4] = - pos[4] # flip theta
        pos[0] = flip_hand_tool[pos[0]]
    with actuator_lock:
        node.publish('/{}-arm/setpoint'.format(arm), pos)


def set_waypoint(node, team_color, pos):
    if team_color is 'green':
        pos = flip_table_xytheta(pos)
    with actuator_lock:
        node.publish('/waypoint', pos)


def is_at_position(node, team_color, target_pos):
    while True:
        try:
            node.recv('/position', timeout=0) # flush queue
        except queue.Empty:
            break
    if team_color is 'green':
        target_pos = flip_table_xytheta(target_pos)
    current_pos = node.recv('/position')
    if (np.linalg.norm(np.array(current_pos[:2]) - np.array(target_pos[:2])) > 0.1
        or abs(target_pos[2] - current_pos[2]) > 0.1):
        return False
    else:
        return True


def goto_waypoint(node, team_color, pos):
    set_waypoint(node, team_color, pos)
    while not is_at_position(node, team_color, pos):
        pass


def set_pump(node, team_color, arm, pump, voltage):
    left_pump_dir = {1: -1, 2: 1, 3: -1, 4: 1, 5: -1}
    right_pump_dir = {1: -1, 2: 1, 3: 1, 4: -1, 5: 1}
    pump_dir = {'left': left_pump_dir, 'right': right_pump_dir}

    if team_color is 'green':
        arm = flip_left_right[arm]
        pump = flip_hand_tool[pump]
    with actuator_lock:
        node.call('/actuator/voltage', ['{}-pump-{}'.format(arm, pump), pump_dir[arm][pump] * voltage])


def safe_arm_position(node):
    with actuator_lock:
        node.publish('/left-arm/setpoint', [0, 0.145, 0.04, 0.185, -pi/2])
        node.publish('/right-arm/setpoint', [0, 0.145, -0.04, 0.135, pi/2])
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
    node.call("/actuator/torque", ["left-wheel", 0])
    node.call("/actuator/torque", ["right-wheel", 0])
    while True: # wait to plug start
        start = node.recv('/interface-panel/start')
        if start is False:
            break
    node.call("/actuator/led_set", ["ready", True])
    node.call("/actuator/velocity", ["left-wheel", 0])
    node.call("/actuator/velocity", ["right-wheel", 0])

    while True: # start unplugged
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


def kill_after_90_seconds(node):
    logging.info("kill thread armed")
    time.sleep(89)
    logging.info("90 seconds over; trying lock actuators")
    actuator_lock.acquire() # block everything
    logging.info("actuators locked")
    node.call("/actuator/led_set", ["ready", True])
    while True:
        node.publish('/waypoint', None)
        zero_velocity(node)


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

    wait_for_start(node)
    threading.Thread(target=kill_after_90_seconds, args=(node,)).start()
    logging.info("start!")

    goto_waypoint(node, team_color, [0.4, 0.4, pi/2])

    goto_waypoint(node, team_color, [0.6, 0.65, pi/2])
    time.sleep(5)

    # above cylinder
    move_arm_in_table_frame(node, team_color, 'right', [5, 0.8, 0.9, 0.24, pi/2])
    time.sleep(1)
    move_arm_in_table_frame(node, team_color, 'right', [5, 0.9, 0.6, 0.24, pi/2])
    time.sleep(2)

    # descend
    move_arm_in_table_frame(node, team_color, 'right', [5, 0.9, 0.635, 0.118, pi/2])
    time.sleep(2)

    set_pump(node, team_color, 'right', 5, 15)
    time.sleep(2)
    # grab
    move_arm_in_table_frame(node, team_color, 'right', [5, 0.9, 0.68, 0.118, pi/2])
    time.sleep(2)

    # ascend
    move_arm_in_table_frame(node, team_color, 'right', [5, 0.9, 0.68, 0.18, pi/2])
    time.sleep(2)

    # hand over
    set_pump(node, team_color, 'left', 5, 15)
    move_arm_in_body_frame(node, team_color, 'left', [5, 0.14, -0.01, 0.185, -1.4])
    move_arm_in_body_frame(node, team_color, 'right', [5, 0.14, 0.01, 0.205, 1.6])
    time.sleep(2)
    set_pump(node, team_color, 'right', 5, -12) # push a little
    time.sleep(2)
    set_pump(node, team_color, 'right', 5, 0)

    move_arm_in_table_frame(node, team_color, 'right', [0, 0.9, 0.65, 0.1, pi])
    time.sleep(2)
    set_pump(node, team_color, 'right', 1, 12)
    set_pump(node, team_color, 'right', 2, 12)
    set_pump(node, team_color, 'right', 3, 12)
    set_pump(node, team_color, 'right', 4, 12)
    time.sleep(2)
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.9, 0.65, 0.06, pi])
    time.sleep(2)
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.9, 0.65, 0.15, pi])
    time.sleep(2)

    goto_waypoint(node, team_color, [0.57, 1.2, pi/2])
    time.sleep(5)
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.88, 1.2, 0.15, pi])
    time.sleep(2)
    # place blocks in center
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.88, 1.2, 0.07, pi])
    time.sleep(2)
    set_pump(node, team_color, 'right', 1, 0)
    set_pump(node, team_color, 'right', 2, 0)
    set_pump(node, team_color, 'right', 3, 0)
    set_pump(node, team_color, 'right', 4, 0)

    set_pump(node, team_color, 'left', 5, 0)
    time.sleep(2)

    move_arm_in_table_frame(node, team_color, 'right', [0, 0.88, 1.2, 0.15, pi])

    time.sleep(2)
    safe_arm_position(node)

if __name__ == '__main__':
    main()
