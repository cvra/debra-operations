''' 
1) run python -m zmqmsgbus.tools.bus
2) run python tools/webots_lidar_interface.py
3) run python tools/lidar_viewer.py
4) start webots simulation (https://github.com/cvra/LIDAR-Simulation.git)
'''

import socketserver, socket, collections, math, json
import numpy as np
import zmqmsgbus
import struct

import pdb

header = {'TypeOfCommand': 'sSN',
          'Command': 'LMDscandata',
          'VersionNumber': 1,
          'DeviceNumber': 1,
          'SerialNumber': 'E5A5AD',
          'DeviceStatus1': 0,
          'DeviceStatus2': 0,
          'TelegramCounter': 2718,
          'AngularStepWidth': 3333,
          'NumberOfData': 811,
          'TimeSinceStartup': 195076021,
          'TimeOfTransmission': 195080325}

class MyUDPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        data = self.request[0]
        socket = self.request[1]

        print('Datagram received, len = '+str(len(data)))

        count = len(data) // struct.calcsize('f')
        data = struct.unpack('f' * count, data)

        header['Data'] = data
        node.publish('/lidar/scan', header)


if __name__ == "__main__":
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    HOST, PORT = "localhost", 9999
    server = socketserver.UDPServer((HOST, PORT), MyUDPHandler)
    server.socket.setsockopt( socket.SOL_SOCKET, socket.SO_RCVBUF, 64000)
    server.serve_forever()

