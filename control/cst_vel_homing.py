import zmqmsgbus
import time
import sys
from math import pi, fmod


def normalize_angle(angle):
    angle = fmod(angle, 2*pi)
    if angle > pi:
        return angle - 2*pi
    if angle < -pi:
        return angle + 2*pi
    return angle


def center_between_periodic_angles(a1, a2):
    return a2 + normalize_angle(a1 - a2) / 2


def homing(actuator, velocity, periodic=False):
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    time.sleep(1)
    node.call('/actuator/velocity', [actuator, 0])

    node.call('/actuator/config_update',
              {'actuator': {actuator: {
                  'stream': {
                      'index':        10,
                      'motor_pos':    30
                  }
              }}})

    state = {'vel': 0, 'idx_p': None, 'idx_n': None, 'idx_seq_nbr': float('inf')}

    def vel_cb(topic, msg):
        state['vel'] = msg

    def idx_cb(topic, msg):
        seq = msg[1]
        if seq > state['idx_seq_nbr']:  # new index
            if abs(state['vel']) > velocity * 0.2:
                if state['vel'] > 0:
                    state['idx_p'] = msg[0]
                else:
                    state['idx_n'] = msg[0]
        state['idx_seq_nbr'] = seq

    node.register_message_handler('/actuator/'+actuator+'/velocity', vel_cb)
    node.register_message_handler('/actuator/'+actuator+'/index', idx_cb)

    n = 0.3
    while True:
        node.call('/actuator/velocity', [actuator, velocity])
        time.sleep(n)
        node.call('/actuator/velocity', [actuator, -velocity])
        time.sleep(2*n)
        node.call('/actuator/velocity', [actuator, velocity])
        time.sleep(n)
        if state['idx_p'] is not None and state['idx_n'] is not None:
            break
        n *= 1.5

    if periodic:
        center = center_between_periodic_angles(state['idx_p'], state['idx_n'])
    else:
        center = (state['idx_p'] + state['idx_n']) / 2

    node.call('/actuator/position', [actuator, center])
    node.call('/actuator/config_update',
              {'actuator': {actuator: {
                  'stream': {
                      'index':        0,
                      'motor_pos':    0
                  }
              }}})

    return center


def main():
    print(homing(sys.argv[1], 1, True))

if __name__ == '__main__':
    main()
