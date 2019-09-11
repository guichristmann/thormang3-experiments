#!/usr/bin/env python

import rospy
from robotis_controller_msgs.msg import JointCtrlModule
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,String

import math
import time
from utils import engine


def bin2float(b):
    f = ((float(b) - 4096.0)*2*math.pi/4096) + math.pi
    return f

class talker:
    def __init__(self):
        self.JointName = []
        name2num = { 
                          'gripON_L'  : "gripON_L",
                          'gripOFF_L' : "gripOFF_L",
                          'gripON_R'  : "gripON_R",
                          'gripOFF_R' : "gripOFF_R",
                          'fLL'  : 15,
                          'bLL'  : 16,
                          'fRR'  : 17,
                          'bRR'  : 18,
                          'init' : 44,
                          'PourStart' : 45,
                          'PourEnd'   : 46,
                          'PouringLtoR' : 48,
                          'look_right' : 22,
                          'look_left'  : 23
        }
        self.engine = engine.Engine(name2num)
    
    def run(self):
        
        rospy.init_node('pyAction', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        sleep = rospy.Rate(200) # 10hz


        while not rospy.is_shutdown():
            page = raw_input('Enter page: ')
            if page == "lookR":
                plan = ["init","look_right"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page == "lookL":
                plan = ["init","look_left"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page == "pick":
                plan = ["gripOFF_L","fLL","gripON_L","bLL","gripOFF_R","fRR","gripON_R","bRR"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page == "place":
                plan = ["fLL","gripOFF_L","bLL","gripON_L","fRR","gripOFF_R","bRR","gripON_R"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page == "init":
                plan = ["init"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                
                    
           
            if page == "back":
                plan = ["PouringLtoR","PourStart"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
                time.sleep(1)
               
            if page == "pour":
            
                step = float(raw_input('Enter step: '))
                #cutout = int(raw_input('Enter cutout: '))
                
                plan = ["PourStart"]
                print("Start")
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
                time.sleep(3)
                    
                cutout = int(step)
                
                sj9    = bin2float(2604)
                sj11   = bin2float(2150)
                sj13   = bin2float(1440)
               
                ej9    = bin2float(2111)
                ej11   = bin2float(2299)
                ej13   = bin2float(1614)
                
                dj9    = (ej9 - sj9)/step
                dj11   = (ej11 - sj11)/step
                dj13   = (ej13 - sj13)/step
                
                
                
                j7Name = ["r_arm_sh_p1","r_arm_sh_r","r_arm_sh_p2","r_arm_el_y","r_arm_wr_r","r_arm_wr_y","r_arm_wr_p"]
                
                joint = JointState()
                joint.name     = ["r_arm_wr_r","r_arm_wr_y","r_arm_wr_p"]
                joint.velocity = [0.0,0.0,0.0]
                joint.effort   = [0.0,0.0,0.0]
                
                self.engine.set_direct_control(j7Name)
                time.sleep(0.1)
                
                ts = time.time()
                for i in range(1,cutout+1):
                    
                    joint.position = [sj9 + i*dj9,sj11 + i*dj11,sj13 + i*dj13]
                    self.engine.set_joint(joint)
                    sleep.sleep()
                    
                    
                te = time.time()  
                print("spend time: " + str(te-ts))
                
          
                ### direct to initize state
                joint.position = [sj9,sj11,sj13]
                joint.velocity = [0.0,0.0,0.0]
                joint.effort   = [0.0,0.0,0.0]
                self.engine.set_joint(joint)
                time.sleep(0.5)
                
                ### close let ball fall in the tube
                joint.name     = j7Name
                joint.position = [0.519745,-1.03781,-0.740513,-0.604489,0.836553,0.163716,-0.948318]
                joint.velocity = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                joint.effort   = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                self.engine.set_joint(joint)
                time.sleep(0.2)
                
            rate.sleep()
        

if __name__ == '__main__':
    try:
        t = talker()
        t.run()
    except rospy.ROSInterruptException:
        pass
