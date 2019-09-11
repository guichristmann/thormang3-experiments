#!/usr/bin/env python3
import numpy as np
from DMP.PIDMP import RLDMPs
import matplotlib.pyplot as plt

def func_p(s,e):
    x = np.linspace(0,1,50)
    #y = np.concatenate( [np.linspace(0,0,25) , np.linspace(0,1,25)] )
    y = -(x-0.5)**2
    goal_path = np.zeros([1,50])
    goal_path[0,:] = x

    
    return goal_path.T

def learning(rl):
        
    for i in range(2000):    
        print("ep: ", i)
        cost = np.zeros((rl.n_stochastic,rl.timesteps)) + 1e-8
        costT = np.random.random((rl.n_stochastic)) 
        ### generate the random path
        track = rl.rollout()
        ### thormang3 execute the path
        for i in range(len(track.y)):
            ### detect collision
            
            ### excute
            
            ### calulate cost
            
            pass
        #rl.save_weight("w1")
        #rl.load_weight("w1")

        ### update PI2 
        print("total cost:",np.sum(cost) + np.sum(costT))
        rl.updatePI(cost,costT)
    

goal_path = func_p(0,1)
rl = RLDMPs(target_path = goal_path,n_dmps = 1, n_bfs = 100 , decay = 20, y0 = [0] , goal = [1.],ay=np.ones(1)*10.0)
learning(rl)
