import zmq
import zmqmsgbus
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument('value', type=float)
parser.add_argument('--actuator', '-a',  default='foobar2000',
                    help="Actuator to set (default foobar2000)")

parser.add_argument('--negative', '-n', action='store_true')
parser.add_argument('--create', '-c', action='store_true')
parser.add_argument('--position', '-p', action='store_true')
parser.add_argument('--velocity', '-v', action='store_true')
parser.add_argument('--torque', '-t', action='store_true')

args = parser.parse_args()

bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')
node = zmqmsgbus.Node(bus)


sign = 1
if args.negative:
    sign = -1

time.sleep(1)

if args.position:
    res = node.call('/actuator/position', [args.actuator, sign*args.value])
elif args.velocity:
    res = node.call('/actuator/velocity', [args.actuator, sign*args.value])
elif args.torque:
    res = node.call('/actuator/torque', [args.actuator, sign*args.value])

if not res:
    print("call failed")

print(args.actuator, node.call('/actuator/state', args.actuator))
