#!/usr/bin/env python3
import numpy as np
from DMP.PIDMP import RLDMPs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

dmp_y0 = np.array([-1.52017496,  0.04908739,  1.41433029])
dmp_goal = np.array([-1.50848603,  0.0591503 ,  1.44347592])

def gentraj(name):
   
    load_file_name = name
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

    traj = rl.predict().y
    return traj
    
    
traj_init = gentraj("w_0_2_right_3_100_1000.0_0.01_4")
traj_1 = gentraj("w_3_1_right_3_100_857.375_0.01_4")
traj_2 = gentraj("w_3_2_right_3_100_857.375_0.01_4")
traj_3 = gentraj("w_2_3_right_3_100_902.5_0.01_4")

traj_31 = gentraj("w_3_1_right_3_100_857.375_0.01_3.0")
traj_32 = gentraj("w_1_2_right_3_100_950.0_0.01_3")
traj_21 = gentraj("w_1_1_right_3_100_950.0_0.01_2.0")

"""fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(traj_init[0][:,0],traj_init[0][:,1],traj_init[0][:,2],c='black')
ax.scatter(traj_1[0][:,0],traj_1[0][:,1],traj_1[0][:,2],c='r')
ax.scatter(traj_2[0][:,0],traj_2[0][:,1],traj_2[0][:,2],c='g')
ax.scatter(traj_3[0][:,0],traj_3[0][:,1],traj_3[0][:,2],c='b')
plt.show()"""

x = np.linspace(0,1,len(traj_init[0][:,0]))
plt.scatter(x,traj_init[0][:,0],c='black',label="initial")
plt.scatter(x,traj_1[0][:,0],c='r',label="pour 4 to 1")
plt.scatter(x,traj_2[0][:,0],c='g',label="pour 4 to 2")
plt.scatter(x,traj_3[0][:,0],c='b',label="pour 4 to 3")
plt.xlabel("time(s)")
plt.ylabel("raw (rad)")
plt.legend(loc = 4)
plt.show()

plt.scatter(x,traj_init[0][:,1],c='black',label="initial")
plt.scatter(x,traj_1[0][:,1],c='r',label="pour 4 to 1")
plt.scatter(x,traj_2[0][:,1],c='g',label="pour 4 to 2")
plt.scatter(x,traj_3[0][:,1],c='b',label="pour 4 to 3")
plt.xlabel("time(s)")
plt.ylabel("yaw (rad)")
plt.legend(loc = 4)
plt.show()

plt.scatter(x,traj_init[0][:,2],c='black',label="initial")
plt.scatter(x,traj_1[0][:,2],c='r',label="pour 4 to 1")
plt.scatter(x,traj_2[0][:,2],c='g',label="pour 4 to 2")
plt.scatter(x,traj_3[0][:,2],c='b',label="pour 4 to 3")
plt.xlabel("time(s)")
plt.ylabel("pitch (rad)")
plt.legend(loc = 4)
plt.show()

x = np.linspace(0,1,len(traj_init[0][:,0]))
plt.scatter(x,traj_init[0][:,0],c='black',label="initial")
plt.scatter(x,traj_31[0][:,0],c='r',label="pour 3 to 1")
plt.scatter(x,traj_32[0][:,0],c='g',label="pour 3 to 2")
plt.scatter(x,traj_21[0][:,0],c='b',label="pour 2 to 1")
plt.xlabel("time(s)")
plt.ylabel("raw (rad)")
plt.legend(loc = 4)
plt.show()

plt.scatter(x,traj_init[0][:,1],c='black',label="initial")
plt.scatter(x,traj_31[0][:,1],c='r',label="pour 3 to 1")
plt.scatter(x,traj_32[0][:,1],c='g',label="pour 3 to 2")
plt.scatter(x,traj_21[0][:,1],c='b',label="pour 2 to 1")
plt.xlabel("time(s)")
plt.ylabel("yaw (rad)")
plt.legend(loc = 4)
plt.show()

plt.scatter(x,traj_init[0][:,2],c='black',label="initial")
plt.scatter(x,traj_31[0][:,2],c='r',label="pour 3 to 1")
plt.scatter(x,traj_32[0][:,2],c='g',label="pour 3 to 2")
plt.scatter(x,traj_21[0][:,2],c='b',label="pour 2 to 1")
plt.xlabel("time(s)")
plt.ylabel("pitch (rad)")
plt.legend(loc = 4)
plt.show()






