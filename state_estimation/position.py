import zmqmsgbus
import queue
import time


def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    while True:
        try:
            o = node.recv('/odometry_raw', timeout=0)
            node.publish('/position', o)
        except queue.Empty:
            pass
        time.sleep(0.01)

if __name__ == '__main__':
    main()
