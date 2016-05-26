import zmqmsgbus
import queue
import time
import numpy as np
import math
from math import sin, cos
import threading


def periodic_error(angle):
    angle = math.fmod(angle, 2*math.pi)
    if angle > math.pi:
        return angle - 2*math.pi
    if angle < -math.pi:
        return angle + 2*math.pi
    return angle


def map_odometry_to_table(odometry_pos, offset):
    x = cos(offset[2]) * odometry_pos[0] - sin(offset[2]) * odometry_pos[1] + offset[0]
    y = sin(offset[2]) * odometry_pos[0] + cos(offset[2]) * odometry_pos[1] + offset[1]
    theta = odometry_pos[2] + offset[2]
    return [x, y, theta]


def compute_odometry_offset(table_pos, odometry_pos):
    theta = table_pos[2] - odometry_pos[2]
    x = table_pos[0] - (cos(theta) * odometry_pos[0] - sin(theta) * odometry_pos[1])
    y = table_pos[1] - (sin(theta) * odometry_pos[0] + cos(theta) * odometry_pos[1])
    return [x, y, theta]

def kalman_state_propagation(P, Phi, Q):
    """
        P: state covariance
        Phi: state transition jacobian
        Q: state transition noise
        returns P_k+1
    """
    return Phi * P * Phi.T + Q

def kalman_measurement_update(x, P, z_minus_h_of_x, H, R):
    """
        x: state
        P: state covariance
        z_minus_h_of_x: measurement - measurement prediction
        H: measurement prediction jacobian
        R: measurement noise
    """
    S = H * P * H.T + R
    K = P * H.T * np.linalg.inv(S)
    P = (np.eye(len(x)) - K * H) * P
    x = x + K * z_minus_h_of_x
    return x, P


def get_robot_position_from_lidar(lidar):
    lidar_heading = lidar[2]
    robot_position = np.array(lidar[0:2]) - np.array([cos(lidar_heading), sin(lidar_heading)]) * 0.074
    return [robot_position[0], robot_position[1], lidar_heading]



class PositionEstimator():

    def __init__(self):
        self.position_odometry = np.array([0.0, 0.0, 0.0])
        self.x = np.matrix([0.0, 0.0, 0.0]).T # position and heading offset
        self.P = np.matrix(np.diag([0.1, 0.1, 0.1]))

    def update_odometry(self, odometry):
        self.position_odometry = np.array(odometry)

    def reset(self, pos=[0, 0, 0]):
        self.x = np.matrix(compute_odometry_offset(pos, self.position_odometry)).T

    def update_lidar(self, robot_position):
        # state propagation x_k+1 = x_k + n
        Q = np.matrix(np.diag([0.001, 0.001, 0.001]))
        self.P = kalman_state_propagation(self.P, np.eye(3), Q)
        # measurement update
        z = np.matrix(robot_position).T
        h_of_x = np.matrix(map_odometry_to_table(self.position_odometry, self.x.T.tolist()[0])).T
        z_minus_h_of_x = z - h_of_x
        z_minus_h_of_x[2] = periodic_error(z_minus_h_of_x[2])
        H = np.matrix([[1, 0, -sin(self.x[2])*self.position_odometry[0]-cos(self.x[2])*self.position_odometry[1]],
                       [0, 1, cos(self.x[2])*self.position_odometry[0]-sin(self.x[2])*self.position_odometry[1]],
                       [0, 0, 1]])
        R = np.matrix(np.diag([0.03, 0.03, 0.05]))
        self.x, self.P = kalman_measurement_update(self.x, self.P, z_minus_h_of_x, H, R)

    def get_position(self):
        return map_odometry_to_table(self.position_odometry, self.x.T.tolist()[0])

got_lidar_fix = False

def lidar_fix_led_thread(node):
    global got_lidar_fix
    got_lidar_fix_old = got_lidar_fix

    time.sleep(1)
    node.call('/actuator/led_set', ['debug', True])

    while True:
        if got_lidar_fix_old != got_lidar_fix:
            node.call('/actuator/led_set', ['debug', not got_lidar_fix])
            got_lidar_fix_old = got_lidar_fix
        time.sleep(0.05)


def main():
    global got_lidar_fix
    bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                        pub_addr='ipc://ipc/sink')
    node = zmqmsgbus.Node(bus)

    pose = PositionEstimator()
    o = node.recv('/odometry_raw')
    pose.update_odometry(o)
    pose.reset([0.5, 0.5, np.pi/2])
    pose_lock = threading.Lock()

    def pose_reset_cb(reset_val):
        with pose_lock:
            pose.reset(reset_val)

    node.register_service('/position/reset', pose_reset_cb)

    t = threading.Thread(target=lidar_fix_led_thread, args=(node,))
    t.start()

    while True:
        try:
            o = node.recv('/odometry_raw', timeout=0)
            with pose_lock:
                pose.update_odometry(o)
                node.publish('/position', pose.get_position())
        except queue.Empty:
            pass
        try:
            l = node.recv('/lidar/position', timeout=0)
            with pose_lock:
                robot_position_lidar = get_robot_position_from_lidar(l)
                robot_position_kalman = pose.get_position()
                if (np.linalg.norm(np.array(robot_position_lidar[0:2]) - np.array(robot_position_kalman[0:2])) < 0.2
                    and abs(periodic_error(robot_position_lidar[2] - robot_position_kalman[2])) < 0.1):
                    got_lidar_fix = True
                    pose.update_lidar(robot_position_lidar)
                    node.publish('/position', pose.get_position())
                else:
                    got_lidar_fix = False
        except queue.Empty:
            pass
        time.sleep(0.01)

if __name__ == '__main__':
    main()
