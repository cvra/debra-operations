from cvra_actuatorpub.trajectory_publisher import *
import zmq
import zmqmsgbus
import time
import threading

MASTER_BOARD_MSG_ADDR = ('10.0.10.2', 20000)

def get_state(name):
    try:
        global pub_lock
        with pub_lock:
            state = pub.get_state(name, time.time())
    except(KeyError): # actuator does not exist
        return None

    # format: [pos, vel, acc, torque]
    if isinstance(state, PositionSetpoint):
        return [state.value, None, None, None]

    elif isinstance(state, VelocitySetpoint):
        return [None, state.value, None, None]

    elif isinstance(state, TorqueSetpoint):
        return [None, None, None, state.value]

    elif isinstance(state, VoltageSetpoint):
        return state.value

    return state


def update_voltage(arg):
    if len(arg) != 2:
        return False
    global pub_lock
    global pub
    with pub_lock:
        pub.update_actuator(arg[0], VoltageSetpoint(float(arg[1])))
    return True

def update_position(arg):
    if len(arg) != 2:
        return False
    global pub_lock
    global pub
    with pub_lock:
        pub.update_actuator(arg[0], PositionSetpoint(float(arg[1])))
    return True

def update_velocity(arg):
    if len(arg) != 2:
        return False
    global pub_lock
    global pub
    with pub_lock:
        pub.update_actuator(arg[0], VelocitySetpoint(float(arg[1])))
    return True

def update_torque(arg):
    if len(arg) != 2:
        return False
    global pub_lock
    global pub
    with pub_lock:
        pub.update_actuator(arg[0], TorqueSetpoint(float(arg[1])))
    return True

# format: [name, [start, dt, [[pos, vel, acc, torque], [point], ...]]]
def update_trajectory(arg):
    if len(arg) != 2:
        return False
    name = arg[0]
    traj = arg[1]
    start = traj[0]
    dt = traj[1]
    setpoints = traj[2]
    points = [TrajectoryPoint(position=float(p[0]),
                              speed=float(p[1]),
                              acceleration=float(p[2]),
                              torque=float(p[3]))
              for p in setpoints]
    global pub_lock
    global pub
    with pub_lock:
        pub.update_actuator(arg[0], Trajectory(start=start, dt=dt, points=points))
    return True

def main():
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    global pub
    global pub_lock
    pub = SimpleRPCActuatorPublisher(MASTER_BOARD_MSG_ADDR)
    print('publish messages to {}:{}'.format(*MASTER_BOARD_MSG_ADDR))
    pub_lock = threading.Lock()

    node.register_service('/actuator/state', get_state)
    node.register_service('/actuator/voltage', update_voltage)
    node.register_service('/actuator/position', update_position)
    node.register_service('/actuator/velocity', update_velocity)
    node.register_service('/actuator/torque', update_torque)
    node.register_service('/actuator/trajectory', update_trajectory)

    while 1:
        time.sleep(0.02)
        with pub_lock:
            pub.publish(time.time())

if __name__ == "__main__":
    main()
