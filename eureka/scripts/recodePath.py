#!/usr/bin/env python

import rospy
from robotis_controller_msgs.msg import JointCtrlModule
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,String

import math
import time
import numpy as np
import os
from utils import engine
from pydmps import dmp_discrete


def bin2float(b):
    f = ((float(b) - 4096.0)*2*math.pi/4096) + math.pi
    return f

class RecordPath:
    def __init__(self):
        self.recodeTime = 0.01
        rospy.init_node('RecordPath', anonymous=True)
        self.joint_states = None
        ### init engine
        name2num = { 
                          'gripON_L'  : "gripON_L",
                          'gripOFF_L' : "gripOFF_L",
                          'gripON_R'  : "gripON_R",
                          'gripOFF_R' : "gripOFF_R",
                          'fLL'  : 34,
                          'bLL'  : 35,
                          'fRR'  : 36,
                          'bRR'  : 37,
                          'fML'  : 38,
                          'bML'  : 39,
                          'fMR'  : 40,
                          'bMR' : 41,
                          'init' : 44,
                          'PourStart' : 30,
                          'PourEnd'   : 31,
                          'PouringRtoL' : 32,
                          'PouringLtoR' : 33,
        }
        self.engine = engine.Engine(name2num)
        rospy.Subscriber('/robotis/present_joint_states', JointState, self.readJointState)
        
    ### read present joint states
    def readJointState(self,msg):

        joint_dict = dict(zip(msg.name, msg.position))
        self.joint_states = np.array([ joint_dict["l_arm_sh_p1"],
                                       joint_dict["l_arm_sh_r"],
                                       joint_dict["l_arm_sh_p2"],
                                       joint_dict["l_arm_el_y"],
                                       joint_dict["l_arm_wr_r"],
                                       joint_dict["l_arm_wr_y"],
                                       joint_dict["l_arm_wr_p"] ])
                                       
    
    def run(self):
        
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            page = raw_input('Enter page: ')
            if page == "recode":
                motionPath = []
                plan = ["init",'PourStart',"init"]
                self.engine.setPlan(plan)
                s = time.time()
                while self.engine.isRunning:
                
                    e = time.time()
                    if (e - s) > self.recodeTime:
                        
                        ## recode Path
                        motionPath.append(self.joint_states)
                        s = time.time()
                        self.engine.run()
                        
                    
                motionPath = np.array(motionPath)

                np.save(os.path.dirname(__file__) + "/PathData/path.npy",motionPath)
                
                
            rate.sleep()
            
            if page == "learn":
                jname = ["l_arm_sh_p1","l_arm_sh_r","l_arm_sh_p2","l_arm_el_y","l_arm_wr_r","l_arm_wr_y","l_arm_wr_p"]
                self.engine.set_direct_control(jname)
                motionPath = np.load(os.path.dirname(__file__) + "/PathData/path.npy")
                
                
                
                dmp = dmp_discrete.DMPs_discrete(n_dmps=7,\
                                       n_bfs=500, \
                                       ay=np.ones(7)*10.0,\
                                       dt = 1.0/len(motionPath),\
                                       )
                                       
                
                                  
             
                dmp.imitate_path(motionPath.T, plot=False)
                
                dmp.reset_state()
                dmp.y0 = motionPath[0]
                dmp.goal = motionPath[-1]
                y, dy, ddy = dmp.rollout()
                
                joint = JointState()
                joint.name = jname
                for i in range(len(motionPath)):
                    joint.position = y[i]
                    self.engine.set_joint(joint)
                    time.sleep(self.recodeTime)
                
            if page == "load":
                jname = ["l_arm_sh_p1","l_arm_sh_r","l_arm_sh_p2","l_arm_el_y","l_arm_wr_r","l_arm_wr_y","l_arm_wr_p"]
                joint = JointState()
                joint.name = jname
                
                motionPath = np.load(os.path.dirname(__file__) + "/PathData/path.npy")

                for i in range(len(motionPath)):
                    joint.position = motionPath[i]
                    self.engine.set_joint(joint)
                    time.sleep(self.recodeTime)
        

if __name__ == '__main__':
    try:
        t = RecordPath()
        t.run()
    except rospy.ROSInterruptException:
        pass
