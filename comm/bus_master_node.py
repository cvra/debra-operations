import cvra_rpc.message
import cvra_rpc.service_call
import threading
import socketserver
import msgpack
import zmq
import zmqmsgbus
import time

MESSAGE_FILTER = ['wheelbase_waypoint']
SERVICE_FILTER = ['config_update', 'actuator_create_driver', 'led_set']

MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)
MASTER_BOARD_SERVICE_ADDR = ('10.0.10.2', 20001)
MASTER_BOARD_MSG_ADDR = ('10.0.10.2', 20000)

bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')
node = zmqmsgbus.Node(bus)

def message_handler_cb(topic, msg):
    cvra_rpc.message.send(MASTER_BOARD_MSG_ADDR, topic, msg)

def publish_msg_to_zmq_cb(todo, msg, args):
    # print('receiving:', msg, args)
    global node
    node.publish('/'+msg, args)

# service calls zmq -> cvra_rpc
for service in SERVICE_FILTER:
    node.register_service('/actuator/' + service,
                          lambda msg, service=service: cvra_rpc.service_call.call(MASTER_BOARD_SERVICE_ADDR, service, msg))

# messages zmq -> cvra_rpc
for topic in MESSAGE_FILTER:
    node.register_message_handler(topic, message_handler_cb)

# stream cvra_rpc -> zmq
RequestHandler = cvra_rpc.message.create_request_handler({}, publish_msg_to_zmq_cb)
msg_server = socketserver.UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_pub_thd = threading.Thread(target=msg_server.serve_forever)
msg_pub_thd.daemon = True
msg_pub_thd.start()

while True:
    time.sleep(1)
