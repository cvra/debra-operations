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

def is_almost_at_position(node, team_color, target_pos, dist):
    while True:
        try:
            node.recv('/position', timeout=0) # flush queue
        except queue.Empty:
            break
    if team_color is 'green':
        target_pos = flip_table_xytheta(target_pos)
    current_pos = node.recv('/position')
    if (np.linalg.norm(np.array(current_pos[:2]) - np.array(target_pos[:2])) > dist):
        return False
    else:
        return True

# [[x,y,theta], [x,y,theta], ...]
# theta can be None
def follow_trajectory(node, team_color, traj):
    for pos in traj:
        set_waypoint(node, team_color, pos)
        while not is_almost_at_position(node, team_color, pos, 0.1):
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
    zero_velocity(node)
    node.call("/arm/run_zero_homing", None)
    zero_torque(node)
    safe_arm_position(node)

    team_color = get_start_color(node)
    logging.info(team_color)

    pos = [0.1025,0.129,-pi/2]
    logging.debug('resetting to position: {}'.format(pos))
    node.call("/position/reset", pos)
    node.call("/actuator/led_set", [button_and_led_color[team_color] + '_2', True])

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
        set_pump(node, 'violet', 'left', 1, 0)
        set_pump(node, 'violet', 'left', 2, 0)
        set_pump(node, 'violet', 'left', 3, 0)
        set_pump(node, 'violet', 'left', 4, 0)
        set_pump(node, 'violet', 'left', 5, 0)
        set_pump(node, 'violet', 'right', 1, 0)
        set_pump(node, 'violet', 'right', 2, 0)
        set_pump(node, 'violet', 'right', 3, 0)
        set_pump(node, 'violet', 'right', 4, 0)
        set_pump(node, 'violet', 'right', 5, 0)


def main():
    logging.basicConfig(level=logging.DEBUG)

    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    time.sleep(4)

    logging.info("ready")

    zero_torque(node)

    team_color = None
    while team_color is None:
        team_color = init_sequence(node)

    wait_for_start(node)
    threading.Thread(target=kill_after_90_seconds, args=(node,)).start()
    logging.info("start!")

    # move aside box
    follow_trajectory(node, team_color, [[0.5, 0.5, None], [0.5, 0.7, None], [0.4, 0.9, None], [0.3, 1.1, None]])
    goto_waypoint(node, team_color, [0.20, 1.17, pi])
    time.sleep(2)

    # position arm
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.20, 1.47, 0.15, 0])
    set_pump(node, team_color, 'right', 1, 12)
    set_pump(node, team_color, 'right', 2, 12)
    set_pump(node, team_color, 'right', 3, 12)
    set_pump(node, team_color, 'right', 4, 12)
    time.sleep(2)
    # descend
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.20, 1.47, 0.106, 0])
    time.sleep(1)
    # move up
    move_arm_in_table_frame(node, team_color, 'right', [0, 0.20, 1.47, 0.19, 0])
    time.sleep(0.5)

    # move arm with box behind
    move_arm_in_body_frame(node, team_color, 'right', [0, -.19, -0.05, 0.21, pi/2])
    time.sleep(0.5)

    # # move home
    # goto_waypoint(node, team_color, [0.3, 0.5, pi])

    # drop all elements
    time.sleep(10)
    set_pump(node, team_color, 'left', 1, 0)
    set_pump(node, team_color, 'left', 2, 0)
    set_pump(node, team_color, 'left', 3, 0)
    set_pump(node, team_color, 'left', 4, 0)
    set_pump(node, team_color, 'right', 1, 0)
    set_pump(node, team_color, 'right', 2, 0)
    set_pump(node, team_color, 'right', 3, 0)
    set_pump(node, team_color, 'right', 4, 0)


if __name__ == '__main__':
    main()
