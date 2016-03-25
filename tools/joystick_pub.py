import zmq
import zmqmsgbus
import time
import queue
import sys

bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')
node = zmqmsgbus.Node(bus)

time.sleep(1)

FORWARD_SPEED_MAX = 10
ROTATION_SPEED_MAX = 5

while True:
    try:
        axis, buttons = tuple(node.recv('/joystick', 0.5))
    except queue.Empty:
        print('timeout')
        res = node.call('/actuator/velocity', ['left-wheel', 0])
        res = node.call('/actuator/velocity', ['right-wheel', 0])
        sys.exit(1)

    # limit
    axis = list(map(lambda x: -1 if x < -1 else 1 if x > 1 else x, axis))
    # threshold
    axis = list(map(lambda x: x if abs(x) > 0.05 else 0, axis))

    if buttons[9] == 0:
        value = [0,0]
    else:
        value = [FORWARD_SPEED_MAX*axis[1] - ROTATION_SPEED_MAX*axis[0],
                 FORWARD_SPEED_MAX*axis[1] + ROTATION_SPEED_MAX*axis[0]]
        print(value)

    res = node.call('/actuator/velocity', ['left-wheel', value[0]])
    res = node.call('/actuator/velocity', ['right-wheel', -value[1]])
