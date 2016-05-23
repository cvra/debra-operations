import zmqmsgbus
import sys
import yaml
import time

bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')
node = zmqmsgbus.Node(bus)

for i in range(10):
    a = yaml.load(sys.argv[2])
    print(a)
    node.publish(sys.argv[1], a)
    time.sleep(0.01)
