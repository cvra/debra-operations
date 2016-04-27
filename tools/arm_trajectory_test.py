from arm_trajectories.scara import *
import arm_trajectories.spline as spline
import arm_trajectories.trajectory as trajectory
import zmqmsgbus
import time

bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')
node = zmqmsgbus.Node(bus)

time.sleep(1) # time needed to find service call

a = 140
b = 52
limits = [[-3.14/180*120, 3.14/180*120], [-3.14/180*125, 3.14/180*125]]

points = [[192, 0], [140, 52], [100, 100], [100, 150], [0, 192]]

points_inv = [inverse_kinematics(p, a, b, limits) for p in points]

traj = spline.SplineTrajectory(points_inv, start_dir=None, end_dir=None, roundness=0.8)

def actuator_limits(position, velocity):
    return ([-0.5, -0.5], [0.5, 0.5])

delta_t = 0.05
# [[[pos1, pos2], [vel1, vel2], [acc1, acc2]], [pos, vel, acc], [point], ...]
tp = trajectory.compute_trajectory(traj, actuator_limits, 1, delta_t, v_limit=20)

# [[pos, vel, acc, torque], [point], ...]
shoulder =
elbow =

start = time.time()
res = node.call('/actuator/trajectory', ['right-shoulder', [start, delta_t, shoulder]])
print(res)
res = node.call('/actuator/trajectory', ['right-elbow', [start, delta_t, elbow]])
print(res)
