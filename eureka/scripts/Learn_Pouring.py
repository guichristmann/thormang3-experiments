#!/usr/bin/env python

import numpy as np
import os
import time
from time import sleep
import rospy
from std_msgs.msg import Int32,String
from utils.engine import Engine
from utils.mathtool import *
from robotis_controller_msgs.msg import JointCtrlModule
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from eureka.srv import CalFK

from thormang3_manipulation_module_msgs.msg import JointPose, KinematicsPose
from thormang3_manipulation_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse, GetKinematicsPoseRequest
from sensor_msgs.msg import JointState

from DMP.PIDMP import RLDMPs

def genPath(path,r_start,r_end,y_start,y_end,p_start,p_end,step):
    d_r = (r_end - r_start)/step
    d_y = (y_end - y_start)/step
    d_p = (p_end - p_start)/step
    
    for i in range(step):
        path.append([r_start+i*d_r, y_start+i*d_y, p_start+i*d_p])


class LearningPouring:
    def __init__(self):
    
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
                          'DMPPourRtoL' : 28,
                          'removeBall' : 29
                          
        }
        
        self.engine = Engine(name2num)

        self.dmp_y0 = np.array([-1.52017496,  0.04908739,  1.41433029])
        self.dmp_goal = np.array([-1.50848603,  0.0591503 ,  1.44347592])

        load_file_name = "w_0_1_right_3_100_1000.0_0.01_2"
        #load_file_name = raw_input('file name: ')
        load_file_name_list = load_file_name.split('_')
        ### learning ep
        self.ep = int(load_file_name_list[1])
        ### pouring number of ball to the other tube
        self.numofball = int(load_file_name_list[2])
        ### which arm do the pouring motion
        self.pour_arm = load_file_name_list[3]
        n_dmps = int(load_file_name_list[4])
        n_bfs = int(load_file_name_list[5])
        decay = float(load_file_name_list[6])
        dt = float(load_file_name_list[7])
        self.total_ball = float(load_file_name_list[8])

        ### initial DMP
        self.rl = RLDMPs(n_dmps = n_dmps , n_bfs = n_bfs , decay = decay, y0 = self.dmp_y0 , goal = self.dmp_goal,ay=np.ones(n_dmps)*10.0,dt = dt)
            
        self.rl.load_weight(load_file_name)
        
        print(self.rl.predict().y)
        print("load npy file weight success:")
        print("ep: " + str(self.ep))
        print("pouring " + str(self.numofball) + " ball to other tube. Total: " + str(self.total_ball))
        print("using  " + self.pour_arm +" pouring the ball")
        
        self.costT_list = []
        
    def initPublishers(self):
        self.pub_joint_ctrl_module = rospy.Publisher('/robotis/set_joint_ctrl_modules', JointCtrlModule, queue_size=10)
        self.pub_action = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=10)
        self.pub_IK = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg', KinematicsPose, queue_size=1)
        self.pub_joint_value = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        
        self.fk_pub = rospy.Publisher('/thormang3/fk_set_joint_states', JointState, queue_size=10)
        
        # Wait a bit for the publishers to initialize
        sleep(1)
        
    def initSubscribe(self):
        #rospy.Subscriber('/robotis/present_joint_states', JointState, self.callback)
        pass
        
        
    #def callback(self,msg):
    #    self.joint_pose = dict(zip(msg.name, msg.position))
        
    # calulate Ik from robotis IK engine
    def cal_IK(self,name,x,y,z,qx,qy,qz,qw):
        
        self.set_manipulation_module()
        sleep(0.5)
        
        pose_msg = KinematicsPose()
        pose_msg.name = name
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pub_IK.publish(pose_msg)
        
    def get_tube_position(self,arm_type,j):
        rospy.wait_for_service("/thormang3_eureka/cal_fk")
        
        # Create service object
        fk_srv = rospy.ServiceProxy("/thormang3_eureka/cal_fk", CalFK)
        
        try:
            # Call service and get response
            fk_resp = fk_srv(arm_type,j[0],j[1],j[2],j[3],j[4],j[5],j[6])
        except rospy.ServiceException as exc:
            print("Failed to call service: " + str(exc))
            
        oR, oP, oY = euler_from_quaternion([fk_resp.ox, fk_resp.oy, fk_resp.oz, fk_resp.ow])

        Rx = np.array([[1,0,0,0],
                       [0,np.cos(-oR),np.sin(-oR),0],
                       [0,np.sin(-oR),np.cos(-oR),0],
                       [0,0,0,1]])
                       
        Ry = np.array([[ np.cos(oP),0,np.sin(oP),0],
                       [0,1,0,0],
                       [-np.sin(oP),0,np.cos(oP),0],
                       [0,0,0,1]])
                       
        Rz = np.array([[ np.cos(oY),-np.sin(oY),0,0],
                       [ np.sin(oY), np.cos(oY),0,0],
                       [0,0,1,0],
                       [0,0,0,1]])
                       
                       
        bottom = np.array([fk_resp.px,fk_resp.py,fk_resp.pz])
        up = np.array([fk_resp.px,fk_resp.py,fk_resp.pz])
        
        turb_lenght = 0.07
        
        pos = np.array([0,0,turb_lenght,1])
        # rotate j4 
        pos = Rx.dot(pos) 
        # rotate j5
        pos = Ry.dot(pos)
        # rotate j6
        pos = Rz.dot(pos)
        up[0] += pos[0]
        up[1] += pos[1]
        up[2] += pos[2]
        return bottom,up
        
    def detect_collision(self,pose,name):
        
        dic_pos = dict(zip(name,pose))

        right_arm_name = ["r_arm_sh_p1","r_arm_sh_r","r_arm_sh_p2","r_arm_el_y","r_arm_wr_r","r_arm_wr_y","r_arm_wr_p"]
        right_arm_pose = [bin2float(3124),bin2float(1614),bin2float(1029),bin2float(1525),bin2float(1063),bin2float(2084),bin2float(2987)]
        r_pose = dict(zip(right_arm_name,right_arm_pose))
        j = []
        for n in right_arm_name:
            if n in name:
                j.append(dic_pos[n])
            else:
                j.append(r_pose[n])

        #j = [self.joint_pose["r_arm_sh_p1"],self.joint_pose["r_arm_sh_r"],self.joint_pose["r_arm_sh_p2"],self.joint_pose["r_arm_el_y"],self.joint_pose["r_arm_wr_r"],self.joint_pose["r_arm_wr_y"],self.joint_pose["r_arm_wr_p"]]
        # Create service object
            
        # Rb is the point bottom of the right tube, Rt is the point top of the right tube  
        Rb,Rt = self.get_tube_position("right_arm",j)
        
        
        left_arm_pose = [bin2float(1650),bin2float(3129),bin2float(2560),bin2float(2343),bin2float(3543),bin2float(2137),bin2float(1167)]
        
        # Lb is the point bottom of the left tube, Lt is the point top of the left tube  
        Lb,Lt = self.get_tube_position("left_arm",left_arm_pose)

        
        pA,pB,dis = closestDistanceBetweenLines(Rb,Rt,Lb,Lt,clampAll=True)

        return dis
        
        
    def execute_path(self,joint_name,traj,delay_time = 0.0025):
        assert len(joint_name) == len(traj[0]) 
        
        
        joint = JointState()
        joint.name     = joint_name
        joint.velocity = [0.0 for _ in range(len(joint_name))]
        joint.effort   = [0.0 for _ in range(len(joint_name))]
        
        self.set_none_module()
        
        start = time.time()
        for i in range(len(traj)):
            ts = time.time()
            joint.position = traj[i]
            self.pub_joint_value.publish(joint)
            te = time.time()
            while (te - ts) < delay_time:
                te = time.time()
                    
        end = time.time()
        print("execute_path time is : ",end - start)
        
        
    def cal_cost(self,bnum,traj):
        cost = np.zeros((self.rl.timesteps)) + 1e-8
        costT = (self.numofball - bnum)**2
        return cost,costT    
        
    def gripperOpen(self,name):
    
        self.set_none_module()
        sleep(0.5)
        # open gripper
        joint_msg = JointState()
        joint_msg.name = [name]
        joint_msg.position = [0.0]
        self.pub_joint_value.publish(joint_msg)
        self.set_manipulation_module()

    def gripperClose(self,name):
        self.set_none_module()
        # close gripper
        joint_msg = JointState()
        joint_msg.name = [name]
        joint_msg.position = [1.0]
        self.pub_joint_value.publish(joint_msg)
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
        
        
    def run(self):
        rospy.init_node('Learning_pouring', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            page = raw_input('Command: ')
            
            if page == "pickRR":
                self.set_action_modules()
                plan = ["init"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
                self.cal_IK(name = "right_arm", x = 0.460,y = -0.180, z = 1.100,qx = 0,qy = 0,qz = 0,qw = 1)
                sleep(3)
                self.cal_IK(name = "right_arm", x = 0.460,y = -0.180, z = 0.89,qx = 0,qy = 0,qz = 0,qw = 1)
                sleep(3)
                
                self.gripperOpen("r_arm_grip")
                
                raw_input("wait the turb")
                
                self.gripperClose("r_arm_grip")
                
                self.cal_IK(name = "right_arm", x = 0.460,y = -0.180, z = 1.100,qx = 0,qy = 0,qz = 0,qw = 1)
                sleep(3)
                
                self.set_action_modules()
                plan = ["init"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page == "pickLL":
                self.set_action_modules()
                plan = ["init"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
                self.cal_IK(name = "left_arm", x = 0.460,y = 0.180, z = 1.100,qx = 0,qy = 0,qz = 0,qw = 1)
                sleep(3)
                self.cal_IK(name = "left_arm", x = 0.460,y = 0.180, z = 0.89,qx = 0,qy = 0,qz = 0,qw = 1)
                sleep(3)
                
                self.gripperOpen("l_arm_grip")
                
                raw_input("wait the turb")
                
                self.gripperClose("l_arm_grip")
                
                self.cal_IK(name = "left_arm", x = 0.460,y = 0.180, z = 1.100,qx = 0,qy = 0,qz = 0,qw = 1)
                sleep(3)
                
                self.set_action_modules()
                plan = ["init"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page=="cost":
                print(np.load(os.path.dirname(__file__) + "/costT.npy"))
                
            if page=="save":
                save_name = 'w_'
                save_name = save_name +  str(self.ep) + '_'
                save_name = save_name +  str(self.numofball)+ '_'
                save_name = save_name +  self.pour_arm+ '_'
                save_name = save_name +  str(self.rl.n_dmps)+ '_'
                save_name = save_name +  str(self.rl.n_bfs)+ '_'
                save_name = save_name +  str(self.rl.decay)+ '_'
                save_name = save_name +  str(self.rl.dt)+'_'
                save_name = save_name +  str(self.total_ball)
                self.rl.save_weight(save_name)
                np.save(os.path.dirname(__file__) + "/costT.npy",np.array(self.costT_list))
                print("save npy file weight success")
                
            ### learning pouring
            if page=="l": 
            
                ### prepare
                plan = ["DMPPourRtoL"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                        
                print("ep: " + str(self.ep))  
                track = self.rl.rollout()     
                cost = np.zeros((self.rl.n_stochastic,self.rl.timesteps)) + 1e-8
                costT = np.zeros((self.rl.n_stochastic)) + 1e-8 
                
                for i in range(self.rl.n_stochastic):
                    raw_input("wait_ball: ")
                    ### detect collision
                    print("random try: " + str(i))
                    #min_dis = 10
                    #for j in range(len(track.y[i])):    
                    #    dis = self.detect_collision(track.y[i][j],["r_arm_wr_y"])   
                    #    if min_dis > dis:
                    #        min_dis = dis
                    min_dis = 1
                    if min_dis > 0.001:
                        
                        ### excute
                        
                        self.execute_path(["r_arm_wr_r","r_arm_wr_y","r_arm_wr_p"],track.y[i])
                        ### calulate cost
                        bnum = float(raw_input("ball number: "))
                        cost[i],costT[i] = self.cal_cost(bnum,track.y[i])
                        if bnum !=0:
                            plan = ["removeBall","DMPPourRtoL"]
                            self.engine.setPlan(plan)
                            while self.engine.isRunning:
                                self.engine.run()
                            sleep(3)
                            
                    else:
                        print("error: min_dis is " , min_dis)
                        raw_input()
                        costT[i] = -1
                        

                   
                print("total cost:",np.sum(cost) + np.sum(costT))
                self.rl.updatePI(cost,costT)
                self.ep += 1
                self.costT_list.append(costT)
                    
                          
                
                
            if page=="p":
                track = self.rl.predict() 
                ### prepare
                plan = ["DMPPourRtoL"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                sleep(2)
                
                ### excute
                self.execute_path(["r_arm_wr_r","r_arm_wr_y","r_arm_wr_p"],track.y[0])
                
                plan = ["DMPPourRtoL"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                
                sleep(2)
                plan = ["removeBall","DMPPourRtoL"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                sleep(3)


       
            if page=="t":
                path = []
                
                wr_r_p0 = bin2float(1057)
                wr_y_p0 = bin2float(2080)
                wr_p_p0 = bin2float(2970)

                wr_r_p1 = bin2float(948)
                wr_y_p1 = bin2float(1918)
                wr_p_p1 = bin2float(2940)

                wr_r_p2 = bin2float(1067)
                wr_y_p2 = bin2float(2090)
                wr_p_p2 = bin2float(2990)

                genPath(path,wr_r_p0,wr_r_p1,wr_y_p0,wr_y_p1,wr_p_p0,wr_p_p1,30)
                genPath(path,wr_r_p1,wr_r_p1,wr_y_p1,wr_y_p1,wr_p_p1,wr_p_p1,20)
                genPath(path,wr_r_p1,wr_r_p2,wr_y_p1,wr_y_p2,wr_p_p1,wr_p_p2,50)
                path = np.array(path)
                self.execute_path(["r_arm_wr_r","r_arm_wr_y","r_arm_wr_p"],path)
                
                plan = ["removeBall","DMPPourRtoL"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                sleep(3)
                            
                    
            if page == "init":        
                plan = ["init"]
                self.engine.setPlan(plan)
                while self.engine.isRunning:
                    self.engine.run()
                    
            if page == "c":
                self.cal_turb_collision(0)    
            if page == "q":
                break
       

if __name__ == '__main__':
    try:
        t = LearningPouring()
        t.run()
    except rospy.ROSInterruptException:
        pass
