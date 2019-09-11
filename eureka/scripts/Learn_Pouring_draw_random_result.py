#!/usr/bin/env python3
import numpy as np
from DMP.PIDMP import RLDMPs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.random.seed(50)
dmp_y0 = np.array([-1.52017496,  0.04908739,  1.41433029])
dmp_goal = np.array([-1.50848603,  0.0591503 ,  1.44347592])
    

load_file_name = "w_0_2_right_3_100_1000.0_0.01_4"
#load_file_name = raw_input('file name: ')
load_file_name_list = load_file_name.split('_')
### learning ep
ep = int(load_file_name_list[1])
### pouring number of ball to the other tube
numofball = int(load_file_name_list[2])
### which arm do the pouring motion
pour_arm = load_file_name_list[3]
n_dmps = int(load_file_name_list[4])
n_bfs = int(load_file_name_list[5])
decay = float(load_file_name_list[6])
dt = float(load_file_name_list[7])

### initial DMP
rl = RLDMPs(n_dmps = n_dmps , n_bfs = n_bfs , decay = decay, y0 = dmp_y0 , goal = dmp_goal,ay=np.ones(n_dmps)*10.0,dt = dt)

rl.load_weight(load_file_name)

traj_init = rl.predict().y
track = rl.rollout()

print(rl.w)

x = np.linspace(0,1,len(traj_init[0][:,0]))

plt.scatter(x,track.y[0][:,0],c='b',label="random")
plt.scatter(x,track.y[1][:,0],c='b')
plt.scatter(x,track.y[2][:,0],c='b')
plt.scatter(x,track.y[3][:,0],c='b')
plt.scatter(x,track.y[4][:,0],c='b')
plt.scatter(x,traj_init[0][:,0],c='r',label="initial")
plt.xlabel("time(s)")
plt.ylabel("raw (rad)")
plt.legend(loc = 4)
plt.show()


plt.scatter(x,track.y[0][:,1],c='b',label="random")
plt.scatter(x,track.y[1][:,1],c='b')
plt.scatter(x,track.y[2][:,1],c='b')
plt.scatter(x,track.y[3][:,1],c='b')
plt.scatter(x,track.y[4][:,1],c='b')
plt.scatter(x,traj_init[0][:,1],c='r',label="initial")
plt.xlabel("time(s)")
plt.ylabel("yaw (rad)")
plt.legend(loc = 4)
plt.show()


plt.scatter(x,track.y[0][:,2],c='b',label="random")
plt.scatter(x,track.y[1][:,2],c='b')
plt.scatter(x,track.y[2][:,2],c='b')
plt.scatter(x,track.y[3][:,2],c='b')
plt.scatter(x,track.y[4][:,2],c='b')
plt.scatter(x,traj_init[0][:,2],c='r',label="initial")

plt.xlabel("time(s)")
plt.ylabel("pitch (rad)")
plt.legend(loc = 4)
plt.show()







