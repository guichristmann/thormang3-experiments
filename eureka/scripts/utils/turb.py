# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 12:22:02 2018

@author: darwin
"""

import numpy as np

class Turb:
    def __init__(self):
        self.turb = np.zeros([3,4,3])
        self.id = np.zeros([3],dtype=int)
        self.threshold = 20
        self.symble = []
        self.isfindball = False

    def clean(self):
        self.turb = np.zeros([3,4,3])
        self.id = np.zeros([3],dtype=int)
        self.symble = []
        self.isfindball = False

    def add(self,ball,idx):
        self.turb[idx,self.id[idx],:] = ball
        self.id[idx] += 1

    ### combine the cluster if have empty turb
    def combine(self):
        #print(self.turb)
        if abs(self.turb[0,0,0] - self.turb[1,0,0]) < self.threshold:
            for i in range(self.id[1]):
                self.turb[0,self.id[0],:] = self.turb[1,i,:]
                self.turb[1,i,:] = 0
                self.id[0] = self.id[0] + 1
                self.id[1] = self.id[1] - 1
        if abs(self.turb[0,0,0] - self.turb[2,0,0]) < self.threshold:
            for i in range(self.id[2]):
                self.turb[0,self.id[0],:] = self.turb[2,i,:]
                self.turb[2,i,:] = 0
                self.id[0] = self.id[0] + 1
                self.id[2] = self.id[2] - 1
        if abs(self.turb[1,0,0] - self.turb[2,0,0]) < self.threshold:
            for i in range(self.id[2]):
                self.turb[1,self.id[1],:] = self.turb[2,i,:]
                self.turb[2,i,:] = 0
                self.id[1] = self.id[1] + 1
                self.id[2] = self.id[2] - 1
                
    def sort(self):
        nonzero = np.nonzero(self.id)[0]

        ### sort turb by x position
        for i in range(3):
            for j in range(i+1,3):
          
                if self.turb[i,0,0] > self.turb[j,0,0]:
                    tmp = self.turb[i,:,:].copy()
                    self.turb[i,:,:] = self.turb[j,:,:]
                    self.turb[j,:,:] = tmp
                        
        ### sort ball by y position
        for k in range(3):
            for i in range(4):
                for j in range(i+1,4):
                    if self.turb[k,i,1] > self.turb[k,j,1] and self.turb[k,i,1] != 0 and  self.turb[k,j,1] != 0:
                        tmp = self.turb[k,i,:].copy()
                        self.turb[k,i,:] = self.turb[k,j,:]
                        self.turb[k,j,:] = tmp
 
        ### turb is empty
        if len(nonzero) < 3:

            diff = abs(self.turb[1,0,0] - self.turb[2,0,0])
               
            p = np.sum(self.turb[:,:,0])/6.0
            #print(p,diff)

            ### empty middle
            if diff > 190:
                tmp0 = self.turb[0,:,:].copy()
                tmp1 = self.turb[1,:,:].copy()
                self.turb[0,:,:] = tmp1
                self.turb[1,:,:] = tmp0
                
            else:
                ### empty right
                if p < 190:
                    tmp0 = self.turb[0,:,:].copy()
                    tmp1 = self.turb[1,:,:].copy()
                    tmp2 = self.turb[2,:,:].copy()
                    self.turb[0,:,:] = tmp1
                    self.turb[1,:,:] = tmp2
                    self.turb[2,:,:] = tmp0

        ### change to symble
        self.symble = []
        for i in range(3):
            color = []
            for j in range(4):
                if self.turb[i,j,2] == 1:
                    color.append('red')
                elif self.turb[i,j,2] == 2:
                    color.append('green')
                elif self.turb[i,j,2] == 3:
                    color.append('purple')
                else:
                    pass
            self.symble.append(color[::-1])


        print(self.symble)
        self.isfindball = True 










