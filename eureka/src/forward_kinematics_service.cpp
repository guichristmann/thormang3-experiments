#include "ros/ros.h"
#include "eureka/forward_kinematics_service.h"
#include "eureka/CalFK.h"
#include <stdio.h>
#include <iostream>


bool cal(eureka::CalFK::Request  &req,
         eureka::CalFK::Response &res)
{
    // cal right arm FK
    if(req.name == "right_arm")
    {

        
        robotis_->thormang3_link_data_[1]->joint_angle_  = req.j0;
        robotis_->thormang3_link_data_[3]->joint_angle_  = req.j1;
        robotis_->thormang3_link_data_[5]->joint_angle_  = req.j2;
        robotis_->thormang3_link_data_[7]->joint_angle_  = req.j3;
        robotis_->thormang3_link_data_[9]->joint_angle_  = req.j4;
        robotis_->thormang3_link_data_[11]->joint_angle_ = req.j5;
        robotis_->thormang3_link_data_[13]->joint_angle_ = req.j6;
        
        robotis_->calcForwardKinematics(0);
        
        // right arm position
        Eigen::MatrixXd curr_position = robotis_->thormang3_link_data_[35]->position_;
        Eigen::MatrixXd curr_rotation = robotis_->thormang3_link_data_[35]->orientation_;
        Eigen::Quaterniond q = robotis_framework::convertRotationToQuaternion(curr_rotation);

        // publish right arm XYZ position and orientation
        res.px = curr_position(0);
        res.py = curr_position(1);
        res.pz = curr_position(2); 
        res.ow = q.w();
        res.ox = q.x();
        res.oy = q.y();
        res.oz = q.z();

        return true;
    }
    
    // cal left arm FK
    if(req.name == "left_arm")
    {
        robotis_->thormang3_link_data_[2]->joint_angle_  = req.j0;
        robotis_->thormang3_link_data_[4]->joint_angle_  = req.j1;
        robotis_->thormang3_link_data_[6]->joint_angle_  = req.j2;
        robotis_->thormang3_link_data_[8]->joint_angle_  = req.j3;
        robotis_->thormang3_link_data_[10]->joint_angle_  = req.j4;
        robotis_->thormang3_link_data_[12]->joint_angle_ = req.j5;
        robotis_->thormang3_link_data_[14]->joint_angle_ = req.j6;
        
        robotis_->calcForwardKinematics(0);
        
        // left arm position
        Eigen::MatrixXd curr_position = robotis_->thormang3_link_data_[34]->position_;
        Eigen::MatrixXd curr_rotation = robotis_->thormang3_link_data_[34]->orientation_;
        Eigen::Quaterniond q = robotis_framework::convertRotationToQuaternion(curr_rotation);

        // publish right arm XYZ position and orientation
        res.px = curr_position(0);
        res.py = curr_position(1);
        res.pz = curr_position(2); 
        res.ow = q.w();
        res.ox = q.x();
        res.oy = q.y();
        res.oz = q.z();
        return true;
    }
    
    return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fk_thormang_service");
  ros::NodeHandle n;
  
  robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);

  ros::ServiceServer service = n.advertiseService("/thormang3_eureka/cal_fk", cal);
  ROS_INFO("Ready to service cal fk.");
  ros::spin();
  
  
  return 0;
}

