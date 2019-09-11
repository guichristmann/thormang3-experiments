'''
implement PI2 algorithm
from paper: A Generalized Path Integral Control Approach to Reinforcement Learning
link: http://www.jmlr.org/papers/volume11/theodorou10a/theodorou10a.pdf
'''
from dmp import DMPs
import time
import numpy as np

import cv2


 
    
def update_dmp(y,timestep):

    timestepD = int(timestep/2)
    timestepUP = int(timestep/2)
    path = []
    for i in range(timestepD):
        path.append((y[i][0],y[i][1],1))
        path.append((y[i][0],y[i][1],1))
        path.append((y[i][0],y[i][1],1))
        
        
    for i in range(timestepUP):
        #sep = 3
        #diff = y[timestepD+i+1] - y[timestepD+i]
        #for j in range(sep):
        path.append((y[timestepD+i][0],y[timestepD+i][1],0))
        path.append((y[timestepD+i][0],y[timestepD+i][1],0))
        path.append((y[timestepD+i][0],y[timestepD+i][1],0))
        
  
    path.append((y[-1][0],y[-1][1],1))
    
    return path

class TRACK():
    def __init__(self):
        self.clear()
        
    def clear(self):
        self.y        = []
        self.dy       = []
        self.ddy      = []
        self.b        = []
        self.eps = []
        self.int = []
        
        
    def nparray(self):
        self.ddy = np.array(self.ddy)
        self.dy = np.array(self.dy)
        self.y = np.array(self.y)
        self.b = np.array(self.b)
        self.eps = np.array(self.eps)
        self.int = np.array(self.int)


class RLDMPs(DMPs):
    
    def __init__(self, **kwargs):
        
        super(RLDMPs, self).__init__(pattern='discrete', **kwargs)
        
        ### numbers of stochastic noise
        self.n_stochastic = 5
        
        ### recode 
        self.track = TRACK()

       
    def rollout(self):
        
        if self.decay > 1:
            self.decay *= 0.95
            print("decay: ",self.decay)
        else:
            self.decay = 1 
        

        self.track.clear()
        
        for i in range(self.n_stochastic):
            y_track, dy_track, ddy_track, b_track, eps_track,int_track = self.dmp_rollout(has_eps=True)
            
            self.track.y.append(y_track) 
            self.track.dy.append(dy_track) 
            self.track.ddy.append(ddy_track) 
            self.track.b.append(b_track) 
            self.track.eps.append(eps_track) 
            self.track.int.append(int_track) 
        self.track.nparray()

        return self.track


    def predict(self):
        

        self.track.clear()
        
        y_track, dy_track, ddy_track, b_track, eps_track,int_track = self.dmp_rollout(has_eps=False)
        
        self.track.y.append(y_track) 
        self.track.dy.append(dy_track) 
        self.track.ddy.append(ddy_track) 
        self.track.b.append(b_track) 
        self.track.eps.append(eps_track) 
        self.track.int.append(int_track) 
        self.track.nparray()

        return self.track
        
    
        
    def updatePI(self,cost,costT):

        self.reset_state()
        x = self.cs.rollout(tau=1.0, error_coupling=1.0)
        psi = self.gen_psi(x) + 1e-8
        total_cost = np.sum(cost) + np.sum(costT)
        if total_cost > 0.01:
        
            for d in range(self.n_dmps):
                ### calculate the M matrix dimension: self.timesteps x self.n_stochastic
                M = np.zeros((self.n_stochastic,self.timesteps,self.n_bfs,self.n_bfs))
                
                ### g is basis function from the system dynamics
                ### dimension: self.timestep x self.n_stochastic
                
                
                g = np.reshape(self.track.b[:,:,d,:],[self.n_stochastic,self.timesteps,self.n_bfs,1])
                gT = np.reshape(self.track.b[:,:,d,:],[self.n_stochastic,self.timesteps,1,self.n_bfs])
                
                gTg =  np.reshape( np.sum(g * g,axis=2) , [self.n_stochastic,self.timesteps,1,1]) + 1e-8 
                
                
                ggT = np.matmul(g,gT)

                M = ggT/gTg    
                
                ################################################ 
                ### this part calculate the S define from eq(32)
                ################################################

                S = np.zeros((self.n_stochastic,self.timesteps)) + 1e-8

                
                ### immediate cost
                
                acc = np.zeros(self.n_stochastic)
                int_term = np.zeros(self.n_stochastic)
                for i in range(self.timesteps-1,-1,-1):
                    
                    int_term = self.track.int[:,i,d] * 0.0
        
                    S[:,i] += cost[:,i] #+ int_term 
                    S[:,i] += acc 
                    acc    += cost[:,i] #+ int_term 
           

                    ### terminal cost
                    S[:,i] += costT
                    
                print("S: ",np.sum(S))
                S = S.T
                ################################################ 
                ### this part calculate the P define from eq(33)
                ################################################
                h = 10
                maxS = np.reshape( np.max(S,axis=1) ,(self.timesteps,1))
                minS = np.reshape( np.min(S,axis=1) ,(self.timesteps,1))
                
                expS = np.exp(-h * (S - minS) /  (maxS - minS) )

                
                
                P = expS / np.reshape(np.sum(expS,axis=1),(self.timesteps,1))
                
                 
                ###################################################### 
                ### this part calculate the dtheta define from eq(36)
                ######################################################

                PMeps = np.zeros(( self.n_stochastic, self.timesteps, self.n_bfs))
                

                
                
                
                for k in range(self.n_stochastic):
                    R_P = np.reshape(P[:,k],(self.timesteps,1,1))
                    R_eps = np.reshape( self.track.eps[k,:,d,:] ,(self.timesteps,self.n_bfs,1))
                    
                    
                    PMeps[k] = np.matmul(R_P*M[k,:],R_eps )[:,:,0]
                            
                
                dw = np.sum(PMeps,axis=0)
        
                ###################################################### 
                ### this part calculate the dtheta define from eq(38)
                ######################################################
                
                ### normalize through the time
                W = np.ones([self.timesteps,self.n_bfs]) * psi
                up = np.zeros((self.n_bfs))
                down = np.zeros((self.n_bfs))
                N = W.shape[0]

                for i in range(N-1):
                    up += (N - i) * W[i] * dw[i,:]
                    down += (N - i) * W[i]


                    
                dw = up/down

                self.w[d] = self.w[d] + dw
            
            

# ==============================
# Test code
# ==============================
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    rl = RLDMPs(n_dmps = 2, n_bfs = 100 , y0 = [0,0] , goal = [1.,1.],ay=np.ones(2)*10.0)
    rl.learning()
    

