#!/usr/bin/env python3
import numpy as np
from PIDMP import RLDMPs
import os

def bin2float(b):
    f = ((float(b) - 4096.0)*2*np.pi/4096) + np.pi
    return f

def genPath(path,r_start,r_end,y_start,y_end,p_start,p_end,step):
    d_r = (r_end - r_start)/step
    d_y = (y_end - y_start)/step
    d_p = (p_end - p_start)/step
    
    for i in range(step):
        path.append([r_start+i*d_r, y_start+i*d_y, p_start+i*d_p])


### wr_r:1064  wr_y:2070 wr_p:2973 standby
### wr_r:948  wr_y:1918 wr_p:2940 pour
### make teacher path
path = []
wr_r_p0 = bin2float(1057)
wr_y_p0 = bin2float(2080)
wr_p_p0 = bin2float(2970)

wr_r_p1 = bin2float(948)
wr_y_p1 = bin2float(1918)
wr_p_p1 = bin2float(2940)

wr_r_p2 = bin2float(1067)
wr_y_p2 = bin2float(2090)
wr_p_p2 = bin2float(2990)

genPath(path,wr_r_p0,wr_r_p1,wr_y_p0,wr_y_p1,wr_p_p0,wr_p_p1,30)
genPath(path,wr_r_p1,wr_r_p1,wr_y_p1,wr_y_p1,wr_p_p1,wr_p_p1,20)
genPath(path,wr_r_p1,wr_r_p2,wr_y_p1,wr_y_p2,wr_p_p1,wr_p_p2,50)

print(path)
path =np.array(path)
ep = 0
numofball = 2
pour_arm = "right"

### initial DMP
n_dmps = 3
n_bfs = 100
decay = 1.0
dmp_y0 = path[0]
dmp_goal = path[-1]


dt = 0.01
rl = RLDMPs(n_dmps = n_dmps, n_bfs = n_bfs , decay = decay, y0 = dmp_y0 , goal = dmp_goal,ay=np.ones(n_dmps)*10.0 ,dt = dt)
rl.imitate_path(path.T, plot=False)
print(rl.predict().y)
print(dmp_y0,dmp_goal)
save_name = 'w_'
save_name = save_name +  str(ep) + '_'
save_name = save_name +  str(numofball)+ '_'
save_name = save_name +  pour_arm+ '_'
save_name = save_name +  str(n_dmps)+ '_'
save_name = save_name +  str(n_bfs)+ '_'
save_name = save_name +  str(decay)+ '_'
save_name = save_name +  str(dt)

rl.save_weight(save_name)


