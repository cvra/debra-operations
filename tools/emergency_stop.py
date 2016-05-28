import serial
import datagrammessages as dmsg
import struct
import argparse
import cvra_can
from time import sleep

def parse_commandline_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', dest='serial_device',
                        help='Serial port of the CAN interface.',
                        metavar='DEVICE', required=True)
    return parser.parse_args()

def can_dongle(port, bitrate, loopback, silent, power):
    conn = dmsg.SerialConnection(port)
    conn.service_call('silent', silent)
    conn.service_call('bit rate', bitrate)
    conn.service_call('loop back', loopback)
    conn.service_call('bus power', power)
    return conn

class EmergencyStop:
    def __init__(self, port):
        self.conn = can_dongle(port=port, bitrate=1000000, loopback=False, silent=False, power=False)
        self.seq_nb = 0

    def send(self):
        can_id = cvra_can.Frame.ID(0x84E2004, extended=True, remote=False)
        data=struct.pack('B', (self.seq_nb & 0x1f) | 0xc0) # set start & end of transfer bits
        frame = cvra_can.Frame(can_id=can_id, data=data)
        self.conn.service_call('tx', [frame.encode()])
        self.seq_nb = (self.seq_nb + 1) & 0x1f

def main():
    args = parse_commandline_args()
    emergency_stop = EmergencyStop(args.serial_device)
    for i in range(5):
        print('sending EmergencyStop')
        emergency_stop.send()
        sleep(0.01)

if __name__ == '__main__':
    main()