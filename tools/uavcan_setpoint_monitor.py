# if bit7 is set in tailbyte of frame then start of frame
# if start of frame first 7 bits encode the destination node id

import serial
import serial_datagram
import datagrammessages as dmsg
import msgpack
import argparse
import cvra_can
import threading
from sys import exit
from time import sleep

def parse_commandline_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', dest='serial_device',
                        help='Serial port of the CAN interface.',
                        metavar='DEVICE', required=True)
    parser.add_argument('id',
                        help='UAVCAN destination node id.',
                        type=int)
    return parser.parse_args()

trajectory_count = 0
velocity_count = 0
position_count = 0
torque_count = 0
voltage_count = 0
def rx_handler(dest_id, msg):
    t = msg[1]
    frame = cvra_can.Frame.decode(msg[0])
    # is start of frame?
    if frame.data[-1] & 0x80 != 0:
        # is addressed to dest_id?
        if (frame.data[0] & 0xfe)>>1 == dest_id:
            print(frame)
            msg_id = (frame.can_id.value>>8) & 0xffff
            global trajectory_count
            global velocity_count
            global position_count
            global torque_count
            global voltage_count
            if msg_id == 20020:
                trajectory_count += 1
            if msg_id == 20021:
                velocity_count += 1
            if msg_id == 20022:
                position_count += 1
            if msg_id == 20023:
                torque_count += 1
            if msg_id == 20024:
                voltage_count += 1

def err_handler(msg):
    print('[{: 10}]: ERROR FRAME'.format(msg))
    pass

def drop_handler(msg):
    print('[DATA LOSS]')
    pass

def set_filter(conn):
    filter = []

    ids = [20020, # Trajectory
           20021, # Velocity
           20022, # Position
           20023, # Torque
           20024] # Voltage

    # bug: does only work for max 4 filters
    ids = [20020, 20021, 20022, 20024]
    for i in ids:
        filter.append([int(cvra_can.Frame.ID(i<<8, extended=True)),
                           cvra_can.Frame.ID.mask(0xffff80, extended=True)])

    print(filter)

    if not conn.service_call('filter', filter):
        print('filter configuration error')
        sys.exit(1)

def main():
    args = parse_commandline_args()
    conn = dmsg.SerialConnection(args.serial_device)

    conn.service_call('silent', True)
    conn.service_call('bit rate', 1000000)
    conn.service_call('loop back', False)
    conn.service_call('bus power', False)

    set_filter(conn)

    conn.set_msg_handler('rx', lambda msg: rx_handler(args.id, msg))
    conn.set_msg_handler('err', err_handler)
    conn.set_msg_handler('drop', drop_handler)

    print('> connect to "{}"'.format(conn.service_call('name', None)))
    print('> hardware version "{}"'.format(conn.service_call('hw version', None)))
    print('> software version {}'.format(conn.service_call('sw version', None)))
    print('> bus voltage {:.3f}'.format(conn.service_call('bus voltage', None)))

    global trajectory_count
    global velocity_count
    global position_count
    global torque_count
    global voltage_count
    while True:
        sleep(1)
        tra = trajectory_count
        trajectory_count = 0
        vel = velocity_count
        velocity_count = 0
        pos = position_count
        position_count = 0
        tor = torque_count
        torque_count = 0
        vol = voltage_count
        voltage_count = 0
        print('traj {}, vel {}, pos {}, torq {}, vol {}'.format(tra, vel, pos, tor, vol))

if __name__ == '__main__':
    main()