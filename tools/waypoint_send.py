import sys
import zmq
import msgpack
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)

# socket.connect("tcp://localhost:13370")
socket.connect("ipc://ipc/sink")

x = float(sys.argv[1])
y = float(sys.argv[2])

time.sleep(0.1)
socket.send(msgpack.packb('wheelbase_waypoint') + msgpack.packb([x, y], use_single_float=True))
socket.send(msgpack.packb('wheelbase_waypoint') + msgpack.packb([x, y], use_single_float=True))
socket.send(msgpack.packb('wheelbase_waypoint') + msgpack.packb([x, y], use_single_float=True))
