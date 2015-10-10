import cvra_rpc.message
import cvra_rpc.service_call
import threading
import socketserver
import msgpack
import zmq

MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)
MASTER_BOARD_SERVICE_ADDR = ('192.168.3.20', 20001)

context = zmq.Context()

sender = context.socket(zmq.PUB)
# sender.connect("tcp://localhost:13370")
sender.connect("ipc://ipc/sink")


call_handler = context.socket(zmq.REP)
call_handler.bind("ipc://ipc/bus_master_server")
call_handler.bind("tcp://*:5555")


def handle_service_calls():
    while True:
        buf = call_handler.recv()
        name, content = msgpack.unpackb(buf, encoding='ascii')

        res = ['ok', cvra_rpc.service_call.call(MASTER_BOARD_SERVICE_ADDR, name, [content])]

        call_handler.send(msgpack.packb(res))

service_call_thd = threading.Thread(target=handle_service_calls)
service_call_thd.daemon = True
service_call_thd.start()


def publish_msg_to_zmq_cb(todo, msg, args):
    if len(args) == 1:
        args = args[0]
    buf = msgpack.packb(msg) + msgpack.packb(args)
    sender.send(buf)

RequestHandler = cvra_rpc.message.create_request_handler({}, publish_msg_to_zmq_cb)
server = socketserver.UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
server.serve_forever()
