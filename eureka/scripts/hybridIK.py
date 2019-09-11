#!/usr/bin/env python


import rospy
from eureka.msg import JointPosition,JacobianMatrix,eurekaCmd,Position
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from ik_jacobian import *
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel
from robotis_controller_msgs.srv import SetModule
import geometry_msgs.msg
from random import randint
import getpass
from keras.models import load_model
from keras.models import Model
from time import sleep
from utils.engine import Engine
from utils.ik_utils import *

pubListL =  [    'l_arm_sh_p1', # -1.6 to 1.6
                'l_arm_sh_r', # -1.6 to 1.6
                'l_arm_sh_p2', # -1.6 to 1.6
                'l_arm_el_y',  # -1.3 to 1.3
                'l_arm_wr_r', # -2.8 to 2.8
                'l_arm_wr_y', # 1.4 to 1.4
                'l_arm_wr_p', # -1.4 to 1.4                           
            ]

pubListR =  [   'r_arm_sh_p1', # -1.6 to 1.6
                'r_arm_sh_r', # -1.6 to 1.6
                'r_arm_sh_p2', # -1.6 to 1.6
                'r_arm_el_y',  # -1.3 to 1.3
                'r_arm_wr_r', # -2.8 to 2.8
                'r_arm_wr_y', # 1.4 to 1.4
                'r_arm_wr_p', # -1.4 to 1.4                           
            ]

class HybridIK:

    def __init__(self):
        rospy.init_node('hybridIK', anonymous=True)

        self.joint_pose = []   
        self.flag = [False,False,False]
        
        print("init hybridIK")
        self.left_arm_position = []
        self.right_arm_position = []
        self.IK_J = IK_Jacobian()
        self.plan = []

        self.isfindtube = False
        self.isfindball = False
        
        ## Publisher
        self.set_joint_pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        self.fk_pub = rospy.Publisher('/thormang3/fk_set_joint_states', JointState, queue_size=10)
        
        ## Subscriber
        rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
        rospy.Subscriber('/thormang3/eureka_controller', eurekaCmd, self.controller_callback)
        rospy.Subscriber('/thormang3/left_arm_position', JointPosition, self.left_arm_position_callback)
        rospy.Subscriber('/thormang3/right_arm_position', JointPosition, self.right_arm_position_callback)
        
        ## set Subscriber for IK_Jacobian
        rospy.Subscriber('/thormang3/cal_jacobian_finish', JacobianMatrix, self.IK_J.cal_jacobian_finish_callback)
        
        rospy.Subscriber('/thormang3/tube', Position, self.tube_callback)
        
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
        
        self.engine = Engine(name2num)
        
        self.set_module("none")



    def tube_callback(self,msg):

        self.leftP = [msg.Ax,msg.Ay,msg.Az]
        self.midP = [msg.Bx,msg.By,msg.Bz]
        self.rightP = [msg.Cx,msg.Cy,msg.Cz]
        arucoP = [msg.Dx,msg.Dy,msg.Dz]
        
        print(self.leftP)
        print(self.midP )
        print(self.rightP)


        self.fixL = [msg.Dx+0.08,msg.Dy+0.15,msg.Dz]
        self.fixM = [msg.Dx+0.08,msg.Dy,msg.Dz]
        self.fixR = [msg.Dx+0.08,msg.Dy-0.15,msg.Dz]

        self.isfindtube = True


    def controller_callback(self,msg):
        
        
        plan = []
        i = 0
        cmd = msg.command
        while(1):
            L = int(cmd[i])
            plan.append(cmd[i+1:i+1+L])
            i += L+1
            if i == len(cmd):
                break
        
        self.plan = plan        
        print(self.plan)
        self.isfindball = True
        
        
    def controlRobot(self, result,jacob_or_net):
        ### control the robot
        
        if jacob_or_net:
            steps = 40
            curr = []
            for k in result.keys():
                curr.append(self.joint_pose[k])
            
            traj = np.linspace(curr,result.values(),steps)
            
            for i in range(steps):
                joint = JointState()
                joint.name = result.keys()
                joint.position = traj[i]
                self.set_joint_pub.publish(joint)
                sleep(0.01)
            
        else:   
            joint = JointState()
            joint.name = result.keys()
            joint.position = result.values()
            self.set_joint_pub.publish(joint)
            sleep(0.01)


    def getArmEndEffectorCartesianPosition(self, arm_type):
         ### get current end position
        if arm_type == "left_arm":
            curr = self.left_arm_position.copy()
        elif arm_type == "right_arm":
            curr = self.right_arm_position.copy()

        return curr

   
   
    def set_module(self,module):
    
        ### module have 1. none 2. action_module 3 manipulation_module
        try:
            rospy.wait_for_service("/robotis/set_present_ctrl_modules")
            SMService = rospy.ServiceProxy('/robotis/set_present_ctrl_modules', SetModule)
            fk_msg = SMService(module)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def play(self):
        
        if self.isfindball and self.isfindtube:
            self.isfindball = False
            self.isfindtube = False
            raw_input("wait press to play")
            plan = self.plan 
            for p in plan:
                height = 0.90
            
                ### flag
                if self.flag[0]:  ### fixed left tube position
                    #self.leftP = self.fixL
                    pass
                if self.flag[2]:  ### fixed right tube position
                    #self.rightP = self.fixR
                    pass
                if self.flag[0] and self.flag[2]:  ### fixed mid tube position
                    if self.flag[1]:
                        #self.midP = self.fixM
                        pass
            
                print("run: ",p)
                #raw_input("wait press")

                if p == "pickLL":
                    
                    plan = ["gripOFF_L"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move x and y first
                    self.IK(self.leftP[0],self.leftP[1],1.05,"left_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.leftP[0],self.leftP[1],height,"left_arm",True)
                    
                    ### gripON_L
                    #raw_input("gripON_L")
                    plan = ["gripON_L"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.leftP[0],self.leftP[1],1.05,"left_arm",True)
                    
                    ### move to init pose
                    plan = ["init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### record Position
                    
                        
                if p == "placeLL":
                    self.flag[0] = True
                    ### move x and y first
                    self.IK(self.leftP[0],self.leftP[1],1.05,"left_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.leftP[0],self.leftP[1],height,"left_arm",True)
                    
                    ### gripOFF_L
                    #raw_input("gripOFF_L")
                    plan = ["gripOFF_L"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.leftP[0],self.leftP[1],1.05,"left_arm",True)
                    
                    ### move to init pose
                    plan = ["gripON_L","init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                        
                if p == "pickML":
                    
                    plan = ["gripOFF_L"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move x and y first
                    self.IK(self.midP[0],self.midP[1],1.05,"left_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.midP[0],self.midP[1],height,"left_arm",True)
                    
                    ### gripON_L
                    #raw_input("gripON_L")
                    plan = ["gripON_L"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.midP[0],self.midP[1],1.05,"left_arm",True)
                    
                    ### move to init pose
                    plan = ["init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### record Position
                    
                        
                if p == "placeML":
                    self.flag[1] = True
                    ### move x and y first
                    self.IK(self.midP[0],self.midP[1],1.05,"left_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.midP[0],self.midP[1],height,"left_arm",True)
                    
                    ### gripOFF_L
                    #raw_input("gripOFF_L")
                    plan = ["gripOFF_L"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.midP[0],self.midP[1],1.05,"left_arm",True)
                    
                    ### move to init pose
                    plan = ["gripON_L","init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                    
                    
                ##############
                #  right arm
                ##############
                
                if p == "pickRR":
                    
                    plan = ["gripOFF_R"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move x and y first
                    self.IK(self.rightP[0],self.rightP[1],1.05,"right_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.rightP[0],self.rightP[1],height,"right_arm",True)
                    
                    ### gripON_L
                    #raw_input("gripON_R")
                    plan = ["gripON_R"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.rightP[0],self.rightP[1],1.05,"right_arm",True)
                    
                    ### move to init pose
                    plan = ["init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    
                    
                        
                if p == "placeRR":
                    self.flag[2] = True
                    ### move x and y first
                    self.IK(self.rightP[0],self.rightP[1],1.05,"right_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.rightP[0],self.rightP[1],height,"right_arm",True)
                    
                    ### gripOFF_L
                    #raw_input("gripOFF_R")
                    plan = ["gripOFF_R"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.rightP[0],self.rightP[1],1.05,"right_arm",True)
                    
                    ### move to init pose
                    plan = ["gripON_R","init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                        
                if p == "pickMR":
                    
                    plan = ["gripOFF_R"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move x and y first
                    self.IK(self.midP[0],self.midP[1],1.05,"right_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.midP[0],self.midP[1],height,"right_arm",True)
                    
                    ### gripON_L
                    #raw_input("gripON_R")
                    plan = ["gripON_R"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.midP[0],self.midP[1],1.05,"right_arm",True)
                    
                    ### move to init pose
                    plan = ["init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### record Position
                    
                        
                if p == "placeMR":
                    self.flag[1] = True
                    ### move x and y first
                    self.IK(self.midP[0],self.midP[1],1.05,"right_arm",False)
                    
                    ### move z for Jacobain
                    self.IK(self.midP[0],self.midP[1],height,"right_arm",True)
                    
                    ### gripOFF_L
                    #raw_input("gripOFF_L")
                    plan = ["gripOFF_R"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                    ### move -z for Jacobain
                    self.IK(self.midP[0],self.midP[1],1.05,"right_arm",True)
                    
                    ### move to init pose
                    plan = ["gripON_R","init"]
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()  
                        
                if p == "PourStart":  
                    plan = ['PourStart']
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run() 
                        
                if p == "PourEnd":  
                    plan = ["PourEnd",'init']
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run() 
                        
                if p == "pourRtoL":
                    plan = ['PouringRtoL']
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                        
                if p == "pourLtoR":
                    plan = ['PouringLtoR']
                    self.engine.setPlan(plan)
                    while self.engine.isRunning:
                        self.engine.run()
                   
                
                self.set_module("none")
            


    def IK(self,px,py,pz,arm_type,all_Jacobian):
        #self.target_pose = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
        
    	self.set_module("none")

        print(np.array([px,py,pz,arm_type]))
        
        if arm_type == "left_arm" or arm_type == "right_arm":


            step = 0.1
            ### get current end position
            if arm_type == "left_arm":
                curr = self.left_arm_position.copy()
            elif arm_type == "right_arm":
                curr = self.right_arm_position.copy()
                px = px - 0.005
                py = py + 0.020
                
            target = curr.copy()
            target[0] = px
            target[1] = py
            target[2] = pz
            
            jacob_or_net = False # If false, use jacobian, if true use NN
            
            
            #raw_input("Check input value, Press enter to start...")
            i = 0
            stop_counter = 0
            while(True):
                print("Current I: {}".format(i))
                
                
                if arm_type == "left_arm":
                    curr = self.left_arm_position.copy()
                elif arm_type == "right_arm":
                    curr = self.right_arm_position.copy()
                    
                print("Current position of end effector: " , curr)
                
                delta = target - curr
                dis = np.sqrt(delta[0]**2 + delta[1]**2 + delta[2]**2)
                print("Distance to target: ",dis)
                
                
                if all_Jacobian:
                    print("### Using jacobian for this step ###")
                    jacob_or_net = False
                elif dis <= 0.12:
                    print("### Using jacobian for this step ###")
                    jacob_or_net = False
                elif dis > 0.12:
                    print("### Using NN for this step ###")
                    jacob_or_net = True
                    
                
                if dis <= 0.005:
                    step = dis
                    stop_counter += 1
                elif jacob_or_net == False:
                    step = 0.005
                elif jacob_or_net == True:
                    step = 0.1
                    
                    
                
                
                stepvec = delta / dis * step

                
                if stop_counter == 3:
                    print("### Goal position reached ###")
                    break
                
                
                #print("Delta Vector: " ,delta)
                #print("Step Vector: " ,stepvec)
                
                r,p,y = quaternion_to_euler_angle(curr[4], curr[5], curr[6], curr[3])
                
    
                # Makes the ik using the net(True)
                # WE DO NOT CARE ABOUT ORIENTATION RIGHT NOW
                stepvec[3] = -r
                stepvec[4] = -p
                stepvec[5] = -y
                stepvec[6] = 0.0
                
                stepvec = stepvec[:6]
                 
                result = self.IK_J.cal(self.joint_pose, stepvec, arm_type, jacob_or_net)
                
                # Control the robot
                if np.max(stepvec) < 0.8 and np.min(stepvec) > -0.8:
                    self.controlRobot(result,jacob_or_net)
                

                i += 1
                
        else:
            print("The name of the arm type is wrong!")
     
        

    def present_joint_states_callback(self,msg):
        self.joint_pose = dict(zip(msg.name, msg.position))
        self.fk_pub.publish(msg)
        #pass
        #print(self.joint_pose)
        
    def left_arm_position_callback(self,msg):

        self.left_arm_position = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
        #print("left_arm_position: ",self.left_arm_position)
    	
    def right_arm_position_callback(self,msg):
        self.right_arm_position = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
        #print("right_arm_position: ",self.right_arm_position)
    def run(self):
        print("run hybridIK")
        self.rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            self.play()
            self.rate.sleep()
        

if __name__ == '__main__':
    np.set_printoptions(suppress=True) 
    l = HybridIK()
    l.run()
