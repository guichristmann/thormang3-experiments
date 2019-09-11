# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 12:22:02 2018

@author: darwin
"""
import time
import rospy
from std_msgs.msg import Int32,String
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import JointCtrlModule

class Engine():
    def __init__(self,name2num):
        self.plan = []
        self.pub  = None
        self.index = 0
        self.max_index = 0
        self.name2num = name2num
        self.canRun = True
        self.isRunning = False
        self.JointName = []  
        rospy.Subscriber('/robotis/movement_done', String, self.movecallback)
        self.subJointName = rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
        self.pub_set_joint_ctrl_modules = rospy.Publisher('/robotis/set_joint_ctrl_modules', JointCtrlModule, queue_size=10)
        self.pub_action = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=10)
        self.pub_setjoint = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        
        
        
    def movecallback(self,data):
        self.canRun = True
 
    def present_joint_states_callback(self,data):
        #print("read joint name")
        self.JointName = data.name
        self.subJointName.unregister()
        
    def set_direct_control(self,name):
        j = JointCtrlModule()
        j.joint_name = name
        for _ in name:
            j.module_name.append("none")
        self.pub_set_joint_ctrl_modules.publish(j)
                
    def set_joint(self,joint):
        self.pub_setjoint.publish(joint)
    
    def gripON_L(self):
        joint = JointState()
        joint.name = ["l_arm_grip"]
        joint.position = [0.995]
        joint.velocity = [0.0]
        joint.effort   = [0.0]
        self.pub_setjoint.publish(joint)
    def gripOFF_L(self):
        joint = JointState()
        joint.name = ["l_arm_grip"]
        joint.position = [0.5]
        joint.velocity = [0.0]
        joint.effort   = [0.0]
        self.pub_setjoint.publish(joint)

    def gripON_R(self):
        joint = JointState()
        joint.name = ["r_arm_grip"]
        joint.position = [0.995]
        joint.velocity = [0.0]
        joint.effort   = [0.0]
        self.pub_setjoint.publish(joint)
    def gripOFF_R(self):
        joint = JointState()
        joint.name = ["r_arm_grip"]
        joint.position = [0.5]
        joint.velocity = [0.0]
        joint.effort   = [0.0]
        self.pub_setjoint.publish(joint)
    
    
    def setPlan(self,plan):
        self.plan = plan
        self.index = 0
        self.max_index = len(plan)
        self.isRunning = True
        
        # set joint_ctrl_modules
        #print("set joint_ctrl_modules")
        j = JointCtrlModule()
        j.joint_name = self.JointName

        for name in self.JointName:
            if "grip" in name:
                j.module_name.append("none")
            else:
                j.module_name.append("action_module")
        
        self.pub_set_joint_ctrl_modules.publish(j)
        time.sleep(0.5)

    def run(self):
    
        if self.isRunning:
		    if self.index < self.max_index:
		        if self.canRun:
		        
		            move = self.plan[self.index]
		        
		            if move == "gripON_L":
		                self.gripON_L()
		                time.sleep(1)
		            
		            elif move == "gripOFF_L":
		                self.gripOFF_L()
		                time.sleep(1)
		            elif move == "gripON_R":
		                self.gripON_R()
		                time.sleep(1)

		            elif move == "gripOFF_R":
		                self.gripOFF_R()
		                time.sleep(1)

		            else:
		                self.pub_action.publish(Int32(self.name2num[move]))
		                self.canRun = False

		            print("engine run: " + move)
		            self.index += 1
	 
		    else:
				if self.canRun:
				#print("end")
					self.isRunning = False












