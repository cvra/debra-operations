import cvra_rpc.message
import cvra_rpc.service_call
import threading
import socketserver
import msgpack
import zmq

MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)
MASTER_BOARD_SERVICE_ADDR = ('192.168.3.20', 20001)
MASTER_BOARD_MSG_ADDR = ('192.168.3.20', 20001)

context = zmq.Context()

msg_pub_sock = context.socket(zmq.PUB)
# msg_pub_sock.connect("tcp://localhost:13370")
msg_pub_sock.connect("ipc://ipc/sink")

msg_sub_sock = context.socket(zmq.SUB)
# msg_sub_sock.connect("tcp://localhost:13371")
msg_sub_sock.connect("ipc://ipc/source")

call_handler = context.socket(zmq.REP)
call_handler.bind("ipc://ipc/bus_master_server")
call_handler.bind("tcp://*:5555")


def handle_service_calls():
    while True:
        buf = call_handler.recv()
        name, content = msgpack.unpackb(buf, encoding='ascii')

        res = ['ok', cvra_rpc.service_call.call(MASTER_BOARD_SERVICE_ADDR, name, content)]

        call_handler.send(msgpack.packb(res))

service_call_thd = threading.Thread(target=handle_service_calls)
service_call_thd.daemon = True
service_call_thd.start()



def publish_msg_to_zmq_cb(todo, msg, args):
    buf = msgpack.packb(msg) + msgpack.packb(args)
    msg_pub_sock.send(buf)

RequestHandler = cvra_rpc.message.create_request_handler({}, publish_msg_to_zmq_cb)
msg_server = socketserver.UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_pub_thd = threading.Thread(target=msg_server.serve_forever)
msg_pub_thd.daemon = True
msg_pub_thd.start()


ANY_TOPIC = b''
msg_sub_sock.setsockopt(zmq.SUBSCRIBE, ANY_TOPIC)

while True:
    buf = msg_sub_sock.recv()
    unpacker = msgpack.Unpacker(encoding='utf8')
    unpacker.feed(buf)
    msg, arg = tuple(unpacker)
    print(msg, arg)
    cvra_rpc.message.send(MASTER_BOARD_MSG_ADDR, msg, arg)
