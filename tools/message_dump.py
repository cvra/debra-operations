import sys
import zmq
import msgpack

context = zmq.Context()
socket = context.socket(zmq.SUB)

# socket.connect("tcp://localhost:13371")
socket.connect("ipc://ipc/source")
for topic in sys.argv[1:]:
    socket.setsockopt(zmq.SUBSCRIBE, msgpack.packb(topic))
if len(sys.argv) == 1:
    socket.setsockopt(zmq.SUBSCRIBE, b'')

while True:
    buf = socket.recv()
    unpacker = msgpack.Unpacker(encoding='utf8')
    unpacker.feed(buf)
    print(tuple(unpacker))
