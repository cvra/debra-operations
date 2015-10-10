from cvra_rpc.message import *
import socketserver
import msgpack
import zmq


context = zmq.Context()

sender = context.socket(zmq.PUB)
# sender.connect("tcp://localhost:13370")
sender.connect("ipc:///sink")

MASTER_BOARD_ADDR = ('0.0.0.0', 20042)


def publish_msg_to_zmq_cb(todo, msg, args):
    if len(args) == 1:
        args = args[0]
    buf = msgpack.packb(msg) + msgpack.packb(args)
    sender.send(buf)

RequestHandler = create_request_handler({}, publish_msg_to_zmq_cb)
server = socketserver.UDPServer(MASTER_BOARD_ADDR, RequestHandler)
server.serve_forever()
