#!/usr/bin/env python3

import numpy as np
import rospy,rospkg

rospy.init_node('generate_traj_node',anonymous=True)
frequency = rospy.get_param('/airo_control_node/fsm/fsm_frequency')

rospack = rospkg.RosPack()
package_path = rospack.get_path('airo_trajectory')
output_path = package_path + '/scripts/circle.txt'

# Parameters
sample_time = 1/frequency      # seconds
duration = 20                  # seconds

r = 1.8                        # m
v = 2.0                        # m/s
# v = np.arange(1.5,2.5,(2.5-1.5)*sample_time/duration)
# v = np.append(v,2.5)

# Circle Center
x0 = 0.5                
y0 = 0.5
z0 = 1.0

# Trajectory
traj = np.zeros((int(duration/sample_time+1),10)) #x y z u v w du dv dw psi
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = -r*np.cos(t*v/r)+x0
traj[:,1] = -r*np.sin(t*v/r)+y0
traj[:,2] = z0
traj[:,3] = v*np.sin(t*v/r)
traj[:,4] = -v*np.cos(t*v/r)
traj[:,5] = 0.0
traj[:,6] = v*v/r*np.cos(t*v/r)
traj[:,7] = v*v/r*np.sin(t*v/r)
traj[:,8] = 0.0
traj[:,9] = np.arctan2(np.sin(t*v/r),np.cos(t*v/r))
# traj[:,9] = 0.0

# write to txt
np.savetxt(output_path,traj,fmt='%f')
print("circle.txt updated!")