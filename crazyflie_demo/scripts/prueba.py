import numpy as np
from trajectory import min_snap_trajectory

def hover_line(rise, start_pt=[0,0,0]):
    return np.array([start_pt, [start_pt[0], start_pt[1], start_pt[2]+rise]])

trajectory_function = min_snap_trajectory

wayptset = hover_line(2)

traj_vars = trajectory_function(0,10,None, waypts = wayptset)

for i in range(30):
    [pos, vel, acc, yaw, yawdot] = trajectory_function(i, 10, traj_vars)
    print(pos,vel,i)