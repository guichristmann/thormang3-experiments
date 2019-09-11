#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32,String
from robotis_controller_msgs.msg import JointCtrlModule
from thormang3_manipulation_module_msgs.msg import JointPose, KinematicsPose
from thormang3_manipulation_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse, GetKinematicsPoseRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from time import sleep
from sensor_msgs.msg import JointState
import random

#### Steps ####
# 1. Set all joints to manipulation ctrl module
# 2. Set gripper to gripper ctrl module
# 3. Move left arm (that holds the bow) of robot to initial pose publishing on /robotis/manipulation/joint_pose_msg. Record initial values from current position of robot.
# 4. Use IK to move the right arm of the robot. To use Robotis' Jacobian publish on /robotis/manipulation/kinematics_pose_msg

# The distance the right arm will pull the bow
PULL_DIST = 0.50 # In meters

class Archery:
    def __init__(self):
        rospy.init_node("archery_core")

        self.arm_Joint = [ "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_p",
                           "l_arm_wr_r", "l_arm_wr_y", "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_p",
                           "r_arm_wr_r", "r_arm_wr_y"]
                           
        self.leg_Joint = ["l_leg_an_p", "l_leg_an_r", "l_leg_hip_p", "l_leg_hip_r", "l_leg_hip_y", "l_leg_kn_p", "r_arm_el_y", "r_leg_an_p", "r_leg_an_r", "r_leg_hip_p", "r_leg_hip_r", "r_leg_hip_y","r_leg_kn_p", "torso_y"]
                                                
        self.grip_Joint = [ "l_arm_grip", "r_arm_grip"]
         
        # Initialize publisher objects
        self.initPublishers()
        
    def initPublishers(self):
        self.pub_joint_ctrl_module = rospy.Publisher('/robotis/set_joint_ctrl_modules', JointCtrlModule, queue_size=10)
        self.pub_action = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=10)
        self.pub_IK = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg', KinematicsPose, queue_size=1)
        self.pub_joint_value = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        
        # Wait a bit for the publishers to initialize
        sleep(1)
        
    def resetJoints(self):
        print("[resetJoints]")
        self.set_action_modules()
        # Set all relevant joints to initial positions
        self.pub_action.publish(Int32(55))
        sleep(3)
        self.set_manipulation_module()
        
    def initPose(self):
        print("[initPose]")
        self.set_action_modules()
        # Set all relevant joints to initial positions
        self.pub_action.publish(Int32(1))
        sleep(3)
        self.set_manipulation_module()
        
    def set_action_modules(self):
        # Set arm to manipulation module
        j = JointCtrlModule()
        j.joint_name = self.arm_Joint + self.leg_Joint
        j.module_name = [ "action_module" for _ in range(len(j.joint_name))]
        self.pub_joint_ctrl_module.publish(j)
        
        # Wait a bit for the publishers to set_joint_ctrl_modules
        sleep(0.2)

    def set_manipulation_module(self):
    
    
        # Set arm to manipulation module
        j = JointCtrlModule()
        j.joint_name = self.arm_Joint
        j.module_name = [ "manipulation_module" for _ in range(len(self.arm_Joint))]
        self.pub_joint_ctrl_module.publish(j)
        
        # Wait a bit for the publishers to set_joint_ctrl_modules
        sleep(0.2)
        
        # Set gripper to gripper module
        j = JointCtrlModule()
        j.joint_name = self.grip_Joint
        j.module_name = [ "gripper_module" for _ in range(len(self.grip_Joint))]
        self.pub_joint_ctrl_module.publish(j)
        
        # Wait a bit for the publishers to set_joint_ctrl_modules
        sleep(0.2)
        
    def set_none_module(self):
    
        # Set arm to none module
        j = JointCtrlModule()
        j.joint_name = self.arm_Joint + self.leg_Joint
        j.module_name = [ "none" for _ in range(len(j.joint_name))]
        self.pub_joint_ctrl_module.publish(j)
        
        # Wait a bit for the publishers to set_joint_ctrl_modules
        sleep(0.2)
        
        # Set gripper to gripper module
        j = JointCtrlModule()
        j.joint_name = self.grip_Joint
        j.module_name = [ "none" for _ in range(len(self.grip_Joint))]
        self.pub_joint_ctrl_module.publish(j)
        
        # Wait a bit for the publishers to set_joint_ctrl_modules
        sleep(0.2)
        
    def getFKResult(self, group):
        rospy.wait_for_service("/robotis/manipulation/get_kinematics_pose")

        # Create service object
        fk_srv = rospy.ServiceProxy("/robotis/manipulation/get_kinematics_pose", GetKinematicsPose)
        try:
            # Create request with passed group
            fk_req = GetKinematicsPoseRequest(group)
            # Call service and get response
            fk_resp = fk_srv(fk_req)
        except rospy.ServiceException as exc:
            print("Failed to call service: " + str(exc))

        # Retrieve orientation information from service response
        qX = fk_resp.group_pose.orientation.x
        qY = fk_resp.group_pose.orientation.y
        qZ = fk_resp.group_pose.orientation.z
        qW = fk_resp.group_pose.orientation.w
        # Convert to euler
        oR, oP, oY = euler_from_quaternion([qX, qY, qZ, qW])

        pose_info = {"pX": fk_resp.group_pose.position.x,
                     "pY": fk_resp.group_pose.position.y,
                     "pZ": fk_resp.group_pose.position.z,
                     "oR": oR,
                     "oP": oP,
                     "oY": oY}

        return pose_info
        
        
    def gripperOpen(self,name):
        self.set_none_module()
        # open gripper
        joint_msg = JointState()
        joint_msg.name = name
        joint_msg.position = [0.0,0.0]
        self.pub_joint_value.publish(joint_msg)
        self.set_manipulation_module()

    def gripperClose(self,name):
        self.set_none_module()
        # close gripper
        joint_msg = JointState()
        joint_msg.name = name
        joint_msg.position = [1.15,1.15]
        self.pub_joint_value.publish(joint_msg)
        self.set_manipulation_module()

    def moveLeftArm(self, offsetYaw, offsetPitch):
        self.set_none_module()
        
        self.center_larm_y = 0.785
        self.center_larm_p = 0.0
        print("Moving l_arm...")
        joint_msg = JointState()
        joint_msg.name = ["l_arm_wr_y","l_arm_wr_p"]
        joint_msg.position = [self.center_larm_y + offsetYaw , self.center_larm_p + offsetPitch]
        
        self.pub_joint_value.publish(joint_msg)
        
        self.set_manipulation_module()
        sleep(1)

    def moveRightArm(self, offsetYaw, offsetPitch,offsetBow):
        # Create message for IK for right arm
        p = KinematicsPose()
        p.name = "right_arm"
        
        center_o_rarm_yaw = np.pi / 2.
        center_o_rarm_pitch = 0.0

        # Get current position of left arm
        #info = self.getFKResult("right_arm")
        #print(info)
        info = self.getFKResult("left_arm")
        print("info: ",info)
        # Calculate new X and Y position for right arm
        dX = np.cos(info["oY"]) * np.cos(-info["oP"]) * PULL_DIST
        dY = np.sin(info["oY"]) * np.cos(-info["oP"]) * PULL_DIST
        dZ = np.sin(-info["oP"]) * PULL_DIST
        
        # Calculate offset Bow dX dY and dZ 
        bY = np.sin(-info["oP"]) * offsetBow
        bZ = np.cos(-info["oP"]) * offsetBow
        print("bY: " , bY , " bZ: ", bZ)
        
        pX = info["pX"] + dX 
        pY = info["pY"] + dY + bY
        pZ = info["pZ"] + dZ + bZ
        print("pX: " , pX , " pY: ", pY, " pZ: ", pZ)
        # Convert new orientation to quaternion
        qX, qY, qZ, qW = quaternion_from_euler(0.0, center_o_rarm_pitch + offsetPitch, center_o_rarm_yaw - offsetYaw)

        p.pose.position.x = pX
        p.pose.position.y = pY
        p.pose.position.z = pZ 
        p.pose.orientation.x = qX
        p.pose.orientation.y = qY
        p.pose.orientation.z = qZ
        p.pose.orientation.w = qW
        self.pub_IK.publish(p)

    def run(self):
    
        # gripperOpen
        self.gripperOpen(["l_arm_grip","r_arm_grip"])
        # Go to init pose
        #self.initPose()

        # go to ready pose
        self.resetJoints()

        # hold the bow
        raw_input("wait the bow")
        self.gripperClose(["l_arm_grip","r_arm_grip"])
        
	
        # Generating some values
        yaw_range = np.linspace(np.radians(-10), np.radians(10), 100)
        pitch_range = np.linspace(np.radians(-10), np.radians(10), 100)

        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            offsetYaw   = np.radians(float(raw_input("offsetYaw: ")))
            offsetPitch = np.radians(float(raw_input("offsetPitch: ")))

            # the constant offset from the bow and arrow
            offsetBow = 0.05
            print("Yaw: ", offsetYaw, "Pitch: ", offsetPitch)
            self.moveLeftArm(offsetYaw, offsetPitch)
            self.moveRightArm(offsetYaw, offsetPitch, offsetBow)
            
            sleep(5)
            self.gripperOpen(["r_arm_grip"])
            #y = random.sample(yaw_range, 1)[0]
            #p = random.sample(pitch_range, 1)[0]
            #print("[Yaw]: ", y, "[Pitch]: ", p)
            #self.moveLeftArm(y, p)

            #self.moveRightArm(y, p)

            #sleep(4)

            rate.sleep()
        
        
if __name__ == '__main__':
    try:
        a = Archery()
        a.run()
    except rospy.ROSInterruptException:
        pass
