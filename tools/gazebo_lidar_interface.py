''' 
1) run python -m zmqmsgbus.tools.bus
2) run python tools/webots_lidar_interface.py
3) run python tools/lidar_viewer.py
4) start webots simulation (https://github.com/cvra/LIDAR-Simulation.git)
'''

import socketserver, socket, collections, math, json
import threading
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

class MyUDPHandler_scan(socketserver.BaseRequestHandler):

    def handle(self):
        data = self.request[0]
        socket = self.request[1]

        count = len(data) // struct.calcsize('f')
        data = struct.unpack('f' * count, data)
        data = np.array(data)

        random_reflection_idx = np.random.random_integers(3, len(data)-1, 5)
        data[random_reflection_idx] *= (1+np.random.rand(5))

        header['Data'] = tuple(data[3:])
        node.publish('/lidar/scan', header)

class MyUDPHandler_position(socketserver.BaseRequestHandler):

    def handle(self):
        data = self.request[0]
        socket = self.request[1]

        count = len(data) // struct.calcsize('f')
        data = struct.unpack('f' * count, data)

        pos = list(data[:2])
        pos.append(2 * np.arcsin(data[5]) * np.sign(data[6]))
        node.publish('/odometry/position', pos)

if __name__ == "__main__":
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    HOST, PORT = "0.0.0.0", 9999
    server_scan = socketserver.UDPServer((HOST, PORT), MyUDPHandler_scan)
    server_scan.socket.setsockopt( socket.SOL_SOCKET, socket.SO_RCVBUF, 64000)
    threading.Thread(target=server_scan.serve_forever, daemon=True).start()

    HOST, PORT = "0.0.0.0", 9998
    server_pos = socketserver.UDPServer((HOST, PORT), MyUDPHandler_position)
    server_pos.socket.setsockopt( socket.SOL_SOCKET, socket.SO_RCVBUF, 64000)
    server_pos.serve_forever()
