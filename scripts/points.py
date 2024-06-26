#!/usr/bin/env python3

import numpy as np
import rospy,rospkg

rospy.init_node('generate_traj_node',anonymous=True)
frequency = rospy.get_param('/airo_control_node/fsm/fsm_frequency')

rospack = rospkg.RosPack()
package_path = rospack.get_path('airo_trajectory')
output_path = package_path + '/scripts/points.txt'

# Parameters
sample_time = 1/frequency      # seconds
cycles = 1
step_interval = 7.5

# points_matrix = np.array([[-1,-1,0.7],[1,-1,0.7],[-1,-1,0.7],[-1,1,0.7],[-1,-1,0.7],[-1,-1,2.7],[-1,-1,0.7]])
points_matrix = np.array([[-1,-1,1.0],[0,-1,1.0]])

# Trajectory
duration = cycles*np.size(points_matrix,0)*step_interval

traj = np.zeros((int(duration/sample_time+1),np.size(points_matrix,1)))

t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

for i in range(1,cycles+1):
    for j in range(1,np.size(points_matrix,0)+1):
        traj_start = (i-1)*np.size(points_matrix,0)*step_interval+(j-1)*step_interval
        traj_end = (i-1)*np.size(points_matrix,0)*step_interval+j*step_interval
        traj[int(traj_start/sample_time):int(traj_end/sample_time),0:np.size(points_matrix,1)] = np.tile(points_matrix[j-1,:],(int(step_interval/sample_time),1))
traj[-1,0:np.size(points_matrix,1)] = traj[-2,0:np.size(points_matrix,1)]

# Write to txt
np.savetxt(output_path,traj,fmt='%f')
print("points.txt updated!")