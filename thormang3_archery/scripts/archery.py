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
from thormang3_archery.msg import target

#### Steps ####
# 1. Set all joints to manipulation ctrl module
# 2. Set gripper to gripper ctrl module
# 3. Move left arm (that holds the bow) of robot to initial pose publishing on /robotis/manipulation/joint_pose_msg. Record initial values from current position of robot.
# 4. Use IK to move the right arm of the robot. To use Robotis' Jacobian publish on /robotis/manipulation/kinematics_pose_msg


class States:
    INIT = -1
    PLACE_BOW = 0
    AIM = 1
    GRAB_STRING = 2
    PULL_STRING1 = 3
    PULL_STRING2 = 4
    RELEASE = 5
    END = 99

class Archery:
    def __init__(self):
        rospy.init_node("archery_core")
        
        ## archery target position in the camera image
        self.targetPos = None

        self.head_Joint = ["head_p","head_y"]
        
        self.arm_Joint = [ "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_p",
                           "l_arm_wr_r", "l_arm_wr_y", "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_p",
                           "r_arm_wr_r", "r_arm_wr_y"]
                           
        self.leg_Joint = ["l_leg_an_p", "l_leg_an_r", "l_leg_hip_p", "l_leg_hip_r", "l_leg_hip_y", "l_leg_kn_p", "r_arm_el_y", "r_leg_an_p", "r_leg_an_r", "r_leg_hip_p", "r_leg_hip_r", "r_leg_hip_y","r_leg_kn_p", "torso_y"]
                                                
        self.grip_Joint = [ "l_arm_grip", "r_arm_grip"]
         
        # Initialize publisher objects
        self.initPublishers()
        
        # Initialize Subscriber objects
        self.initSubscribe()
        
    def initPublishers(self):
        self.pub_joint_ctrl_module = rospy.Publisher('/robotis/set_joint_ctrl_modules', JointCtrlModule, queue_size=10)
        self.pub_action = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=10)
        self.pub_IK = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg', KinematicsPose, queue_size=1)
        self.pub_joint_value = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        
        # Wait a bit for the publishers to initialize
        sleep(1)
        
    def initSubscribe(self):
        rospy.Subscriber('/thormang3/archery_target', target, self.target_callback)
        
    def target_callback(self,msg):
        # Subscriber the position from find archery target vision program
        self.targetPos = [msg.x,msg.y]
        
    def actionPose(self, pose_page):
        print("[resetJoints]")
        # Call requested pose
        self.set_action_modules()
        self.pub_action.publish(Int32(pose_page))
        sleep(3)
        self.set_manipulation_module()
        
    def initPose(self):
        self.gripperOpen(["l_arm_grip"])
        print("[initPose]")
        self.set_action_modules()
        # Set all relevant joints to initial positions
        self.pub_action.publish(Int32(57))
        sleep(3)
        self.set_manipulation_module()
        
    def set_action_modules(self):
        # Set arm to manipulation module
        j = JointCtrlModule()
        j.joint_name = self.arm_Joint + self.leg_Joint + self.head_Joint
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
        if "l_arm_grip" in name:
            #joint_msg.position = [1.0,1.15] #1.15
            joint_msg.position = [1.0,1.5] #1.15
        else:
            joint_msg.position = [1.15,1.15] #1.15
        self.pub_joint_value.publish(joint_msg)
        self.set_manipulation_module()

    # This arm is used for aiming. Right arm aligns itself to shoot correctly
    # xyz is used to change the position of the arm. If it's None then we just 
    # change the orientation
    def moveLeftArm(self, offsetYaw, offsetPitch, xyz=None):
        self.set_manipulation_module()
        
        # First get information on current position and orientation
        curr_pose = self.getFKResult("left_arm")

        yaw = curr_pose["oY"] + offsetYaw
        pitch = curr_pose["oP"] + offsetPitch
        roll = 0.0 # Roll should always be forced to 0

        if xyz is not None:
            x = xyz[0]
            y = xyz[1]
            z = xyz[2]
        else:
            x = curr_pose["pX"]
            y = curr_pose["pY"]
            z = curr_pose["pZ"]
        
        # Convert orientation info to quaternion because that's what the message
        # expects
        qX, qY, qZ, qW = quaternion_from_euler(roll, pitch, yaw)


        # Create pose message and fill information
        pose_msg = KinematicsPose()
        pose_msg.name = "left_arm"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qX
        pose_msg.pose.orientation.y = qY
        pose_msg.pose.orientation.z = qZ
        pose_msg.pose.orientation.w = qW
        self.pub_IK.publish(pose_msg)

        sleep(1)

    def moveRightArm(self, offsetYaw, offsetPitch, offsetBow, pullDist):
        # Create message for IK for right arm
        p = KinematicsPose()
        p.name = "right_arm"
        
        # Center orientation values for left arm. Roll is ignored because it's always forced to be 0
        center_o_rarm_yaw = 0.0
        center_o_rarm_pitch = 0.0

        # Get current position of left arm
        info = self.getFKResult("left_arm")

        # Calculate new X and Y position for right arm
        dX = np.cos(info["oY"]+center_o_rarm_yaw-np.pi/2) * np.cos(-info["oP"]) * pullDist
        dY = np.sin(info["oY"]+center_o_rarm_yaw-np.pi/2) * np.cos(-info["oP"]) * pullDist
        dZ = np.sin(-info["oP"]) * pullDist
        
        # Calculate offset Bow dX dY and dZ 
        bY = np.sin(-info["oP"]) * offsetBow
        bZ = np.cos(-info["oP"]) * offsetBow
        
        pX = info["pX"] + dX 
        pY = info["pY"] + dY + bY
        pZ = info["pZ"] + dZ + bZ

        # Convert new orientation to quaternion
        qX, qY, qZ, qW = quaternion_from_euler(0.0, center_o_rarm_pitch + offsetPitch, center_o_rarm_yaw + np.pi/2 - offsetYaw)

        p.pose.position.x = pX
        p.pose.position.y = pY
        p.pose.position.z = pZ 
        p.pose.orientation.x = qX
        p.pose.orientation.y = qY
        p.pose.orientation.z = qZ
        p.pose.orientation.w = qW
        self.pub_IK.publish(p)

    def run(self):
    
        #self.initPose()
        #sleep(2)
        
        # These values are used for aiming with the left hand (that holds the bow)
        # right hand will follow using IK
        offsetYaw = 0.0 # Aiming values
        offsetPitch = 0.0 # Aiming values
        grabDist = 0.22 # Distance to grab the string
        offsetBow = 0.00 # How much left arm should compensate "upwards"
        pullDist = 0.60 # Distante to pull the bow string
        centerXTarget = 390 # 
        centerYTarget = 285 # 
        kX = -0.001
        kY = -0.001


        # Coordinates for shooting with the left arm (should be a very strong pull)
        l_arm_shoot_pos = (0.264, 0.500, 1.055)
        
        rate = rospy.Rate(60)
        state = States.INIT # Initial state
        while not rospy.is_shutdown():
            if state == States.INIT:
                print("[INIT]")
                # Go to initial archery pose
                self.actionPose(55)
                # Open both grippers
                self.gripperOpen(["l_arm_grip","r_arm_grip"])

                # Next state
                state = States.PLACE_BOW

            elif state == States.PLACE_BOW:
                print("[PLACE_BOW]") 
                raw_input("Press enter to close left gripper.")
                # Close left gripper (to hold the bow)
                self.gripperClose(["l_arm_grip"])

                # Next state 
		raw_input()
                state = States.AIM
                
            elif state == States.AIM:
                
                # 1. Reads information on target
                # 2. Moves left arm, setting offsetPitch and offsetYaw
                
                # wait for thormang detect the target
                print("[AIM]")
                if self.targetPos != None:
                    
                    # calculate the offsetYaw and offsetPitch from targetpos 
                    # TODO
                    offsetX = self.targetPos[0] - centerXTarget
                    offsetYaw = kX * offsetX
                    offsetY = self.targetPos[1] - centerYTarget
                    offsetPitch = kY * offsetY
                    print("offsetX: " , np.degrees(offsetYaw) ," TargetX: " , self.targetPos[0])
                    print("offsetY: " , np.degrees(offsetPitch) ," TargetY: " , self.targetPos[1])
                    offsetPitch = 0.0
                    self.moveLeftArm(offsetYaw, offsetPitch)
                    state = States.GRAB_STRING

            elif state == States.GRAB_STRING:
                print("[GRAB_STRING]")
                raw_input("Press enter to grab string.")
                self.moveRightArm(offsetYaw, offsetPitch, offsetBow, grabDist)
                sleep(4)
                self.gripperClose(["r_arm_grip"])
                sleep(1.5)
                # Next state
                state = States.PULL_STRING1

            elif state == States.PULL_STRING2:
                print("[PULL_STRING2]")
                self.moveRightArm(offsetYaw, offsetPitch, offsetBow, pullDist)
                sleep(4)
                state = States.RELEASE

            elif state == States.PULL_STRING1:
                print("[PULL_STRING1]")
                self.moveLeftArm(offsetYaw, offsetPitch, xyz=l_arm_shoot_pos)
                sleep(4)
                state = States.PULL_STRING2

            elif state == States.RELEASE:
                print("[RELEASE]")
                raw_input("Press enter to release.")
                self.gripperOpen(["r_arm_grip"])
                state = States.END

            elif state == States.END:
                print("[END]")
   
                
        
            rate.sleep()
        
        
if __name__ == '__main__':
    try:
        a = Archery()
        a.run()
    except rospy.ROSInterruptException:
        pass
