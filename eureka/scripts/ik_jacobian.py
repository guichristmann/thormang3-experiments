#!/usr/bin/env python


import rospy
from eureka.msg import JointPosition,JacobianMatrix
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from utils.ik_utils import *
import time
import getpass
from keras.models import load_model
from keras.models import Model
import sys

class IK_Jacobian:

    def __init__(self):
        self.cal_jacobian_pub = rospy.Publisher('/thormang3/cal_jacobian', String, queue_size=10)        
        self.now_cal = False
        self.model_fn = "/home/ntnuerc/catkin_ws/src/ik_thormang3/trained_models/4x256+3x128ReLU-LR0.005-DATASET-0.5-BATCH-2048_1450_0.0033571092382553604_0.0034675660669906587.h5"
        self.model = load_model(self.model_fn)
        self.model._make_predict_function()

        # Its limits for normalization/desnormalization
        # For dataset_5_0.05.csv
        self.highestList = np.array([0.04987197, 0.04978864, 0.04977316, 3.14112379, 0.96412046, \
                                     3.1381618 , 0.51729475, 2.2218, 0.89599821, 1.17479844, \
                                     1.86409277, 0.96549941, 0.38059967, 0.08726625, 0.08726648, \
                                     0.08726617, 0.08726616, 0.08726638, 0.08726638, 0.08726641])
        self.lowestList = np.array([-0.04991784, -0.04989792, -0.04981673, -3.13640507, -0.97724294, \
                                    -3.14156992, -1.76949141,  0.78370129, -0.44029404, -1.20029644, \
                                     0.05140284, -0.42009913, -1.54997519, -0.08726467, -0.08726612, \
                                    -0.08726649, -0.08726502, -0.08726649, -0.08726613, -0.08726631])


      
    def cal_jacobian_finish_callback(self,msg):

        delta = self.delta 
        
       
        if self.net:

            if self.armType == "left_arm":

                predictArray =  np.array([      delta[0], #x
                                            delta[1], #y
                                            delta[2], #z
                                            delta[3], #r
                                            delta[4], #p
                                            delta[5], #y
                                            self.curr_pose['l_arm_sh_p1'],
                                            self.curr_pose['l_arm_sh_r'],
                                            self.curr_pose['l_arm_sh_p2'],
                                            self.curr_pose['l_arm_el_y'],
                                            self.curr_pose['l_arm_wr_r'],
                                            self.curr_pose['l_arm_wr_y'],
                                            self.curr_pose['l_arm_wr_p']                                
                                        ])
            
                # Normalize the predict array
                orig_joint_pos = predictArray.copy()[6:13]
                predictArray = normalizeArray(predictArray)
                print("Input of network:", predictArray)

                # The ANN output
                deltaAngles = (np.array(self.model.predict(predictArray.reshape(1,13)))[0])  
                print("Output of network:", deltaAngles)
                print(deltaAngles.shape)
                # Desnormalize the predicted angles
                #deltaAngles = desnormalizeArray(deltaAngles)
                self.result = {"l_arm_sh_p1": orig_joint_pos[0] + deltaAngles[0],
                               "l_arm_sh_r": orig_joint_pos[1] + deltaAngles[1],
                               "l_arm_sh_p2": orig_joint_pos[2] + deltaAngles[2],
                               "l_arm_el_y": orig_joint_pos[3] + deltaAngles[3],
                               "l_arm_wr_r": orig_joint_pos[4] + deltaAngles[4],
                               "l_arm_wr_y": orig_joint_pos[5] + deltaAngles[5],
                               "l_arm_wr_p": orig_joint_pos[6] + deltaAngles[6]}
                self.now_cal = False
                
            if self.armType == "right_arm":
            
            
                predictArray = np.array([   delta[0], #x
                                           -delta[1], #y
                                            delta[2], #z
                                            delta[3], #r
                                            delta[4], #p
                                            delta[5], #y
                                            -self.curr_pose['r_arm_sh_p1'],
                                            -self.curr_pose['r_arm_sh_r'],
                                            -self.curr_pose['r_arm_sh_p2'],
                                            -self.curr_pose['r_arm_el_y'],
                                            -self.curr_pose['r_arm_wr_r'],
                                            -self.curr_pose['r_arm_wr_y'],
                                            -self.curr_pose['r_arm_wr_p']                                
                                        ])
            
                # Normalize the predict array
                orig_joint_pos = predictArray.copy()[6:13]
                predictArray = normalizeArray(predictArray)
                print("Input of network:", predictArray)

                # The ANN output
                deltaAngles = (np.array(self.model.predict(predictArray.reshape(1,13)))[0])  
                print("Output of network:", deltaAngles)
                print(deltaAngles.shape)
                # Desnormalize the predicted angles
                #deltaAngles = desnormalizeArray(deltaAngles)
                self.result = {"r_arm_sh_p1": -orig_joint_pos[0] - deltaAngles[0],
                               "r_arm_sh_r": -orig_joint_pos[1] - deltaAngles[1],
                               "r_arm_sh_p2": -orig_joint_pos[2] - deltaAngles[2],
                               "r_arm_el_y": -orig_joint_pos[3] - deltaAngles[3],
                               "r_arm_wr_r": -orig_joint_pos[4] - deltaAngles[4],
                               "r_arm_wr_y": -orig_joint_pos[5] - deltaAngles[5],
                               "r_arm_wr_p": -orig_joint_pos[6] - deltaAngles[6]}
                self.now_cal = False
                 
        else:
            if self.armType == "left_arm":
            
            
                predictArray =  np.array([  delta[0], #x
                                            delta[1], #y
                                            delta[2], #z
                                            delta[3], #r
                                            delta[4], #p
                                            delta[5], #y
                                            self.curr_pose['l_arm_sh_p1'],
                                            self.curr_pose['l_arm_sh_r'],
                                            self.curr_pose['l_arm_sh_p2'],
                                            self.curr_pose['l_arm_el_y'],
                                            self.curr_pose['l_arm_wr_r'],
                                            self.curr_pose['l_arm_wr_y'],
                                            self.curr_pose['l_arm_wr_p']                                
                                        ])
                                        
                orig_joint_pos = predictArray.copy()[6:13]
                jacobian = np.zeros(shape=(6,7))
                for i in range(6):
                    for j in range(7):
                        jacobian[i][j] = msg.matrix[j+7*i]
                        
                jacobianTrans = np.dot(jacobian, np.matrix.transpose(jacobian))
                inverseJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(jacobianTrans))
                    
                deltaAngles = np.dot(inverseJacobian, delta)
                
            
                self.result = {"l_arm_sh_p1": orig_joint_pos[0] + deltaAngles[0],
                               "l_arm_sh_r": orig_joint_pos[1] + deltaAngles[1],
                               "l_arm_sh_p2": orig_joint_pos[2] + deltaAngles[2],
                               "l_arm_el_y": orig_joint_pos[3] + deltaAngles[3],
                               "l_arm_wr_r": orig_joint_pos[4] + deltaAngles[4],
                               "l_arm_wr_y": orig_joint_pos[5] + deltaAngles[5],
                               "l_arm_wr_p": orig_joint_pos[6] + deltaAngles[6]}
                self.now_cal = False
                               
            if self.armType == "right_arm":
            
            
                predictArray =  np.array([  delta[0], #x
                                            delta[1], #y
                                            delta[2], #z
                                            delta[3], #r
                                            delta[4], #p
                                            delta[5], #y
                                            self.curr_pose['r_arm_sh_p1'],
                                            self.curr_pose['r_arm_sh_r'],
                                            self.curr_pose['r_arm_sh_p2'],
                                            self.curr_pose['r_arm_el_y'],
                                            self.curr_pose['r_arm_wr_r'],
                                            self.curr_pose['r_arm_wr_y'],
                                            self.curr_pose['r_arm_wr_p']                                
                                        ])
                                        
                orig_joint_pos = predictArray.copy()[6:13]
                jacobian = np.zeros(shape=(6,7))
                for i in range(6):
                    for j in range(7):
                        jacobian[i][j] = msg.matrix[j+7*i]
                        
                jacobianTrans = np.dot(jacobian, np.matrix.transpose(jacobian))
                inverseJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(jacobianTrans))
                    
                deltaAngles = np.dot(inverseJacobian, delta)
                
            
                self.result = {"r_arm_sh_p1": orig_joint_pos[0] + deltaAngles[0],
                               "r_arm_sh_r": orig_joint_pos[1] + deltaAngles[1],
                               "r_arm_sh_p2": orig_joint_pos[2] + deltaAngles[2],
                               "r_arm_el_y": orig_joint_pos[3] + deltaAngles[3],
                               "r_arm_wr_r": orig_joint_pos[4] + deltaAngles[4],
                               "r_arm_wr_y": orig_joint_pos[5] + deltaAngles[5],
                               "r_arm_wr_p": orig_joint_pos[6] + deltaAngles[6]}
                self.now_cal = False

#
#        max_delta = 10 
#        
#       if deltaAngles.max() < max_delta and deltaAngles.min() > -max_delta:
#        
#            endpose = self.curr_pose
#            if self.armType == "left_arm":
#                #self.result = addLeftArmValue2WholeBody(deltaAngles,endpose)
#            elif self.armType == "right_arm":
#                #self.result = addRightArmValue2WholeBody(deltaAngles,endpose)
#            else:
 #               print("wrong arm type")
 #           self.now_cal = False
 #           
 #       else:
            #print("error deltaAngles",deltaAngles)
#            self.result = self.curr_pose
#            self.now_cal = False
        
        
    def cal(self,curr_pose,delta,armType,net):
        self.armType = armType
        self.curr_pose = curr_pose
        self.delta  = delta
    	self.cal_jacobian_pub.publish(armType)
    	self.now_cal = True
        self.net = net
    	
    	### wait for cal jacobain finish
        while self.now_cal:
            pass
            
            
        return self.result
    
   

