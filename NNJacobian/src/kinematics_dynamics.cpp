/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * kinematics_dynamics.cpp
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#include <iostream>
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

namespace thormang3
{

KinematicsDynamics::KinematicsDynamics() {}
KinematicsDynamics::~KinematicsDynamics() {}

KinematicsDynamics::KinematicsDynamics(TreeSelect tree)
{
  for (int id=0; id<=ALL_JOINT_ID; id++)
    thormang3_link_data_[id] = new LinkData();

  if ( tree == WholeBody )
  {
    thormang3_link_data_[0]->name_               =  "base";
    thormang3_link_data_[0]->parent_             =  -1;
    thormang3_link_data_[0]->sibling_            =  -1;
    thormang3_link_data_[0]->child_              =  38;
    thormang3_link_data_[0]->mass_               =  0.0;
    thormang3_link_data_[0]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[0]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[0]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[0]->joint_limit_max_    =  100.0;
    thormang3_link_data_[0]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[0]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    /* ----- passive joint -----*/

    thormang3_link_data_[38]->name_               =  "passive_x";
    thormang3_link_data_[38]->parent_             =  0;
    thormang3_link_data_[38]->sibling_            =  -1;
    thormang3_link_data_[38]->child_              =  39;
    thormang3_link_data_[38]->mass_               =  0.0;
    thormang3_link_data_[38]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[38]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[38]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[38]->joint_limit_max_    =  100.0;
    thormang3_link_data_[38]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[38]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    thormang3_link_data_[39]->name_               =  "passive_y";
    thormang3_link_data_[39]->parent_             =  38;
    thormang3_link_data_[39]->sibling_            =  -1;
    thormang3_link_data_[39]->child_              =  40;
    thormang3_link_data_[39]->mass_               =  0.0;
    thormang3_link_data_[39]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[39]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[39]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[39]->joint_limit_max_    =  100.0;
    thormang3_link_data_[39]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[39]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    thormang3_link_data_[40]->name_               =  "passive_z";
    thormang3_link_data_[40]->parent_             =  39;
    thormang3_link_data_[40]->sibling_            =  -1;
    thormang3_link_data_[40]->child_              =  43;
    thormang3_link_data_[40]->mass_               =  0.0;
    thormang3_link_data_[40]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.723 ); // 0.0 , 0.0 , 0.801
    thormang3_link_data_[40]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[40]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[40]->joint_limit_max_    =  100.0;
    thormang3_link_data_[40]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[40]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    thormang3_link_data_[41]->name_               =  "passive_roll";
    thormang3_link_data_[41]->parent_             =  42;
    thormang3_link_data_[41]->sibling_            =  -1;
    thormang3_link_data_[41]->child_              =  44;
    thormang3_link_data_[41]->mass_               =  0.0;
    thormang3_link_data_[41]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[41]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
    thormang3_link_data_[41]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[41]->joint_limit_max_    =  2.0 * M_PI;
    thormang3_link_data_[41]->joint_limit_min_    =  -2.0 * M_PI;
    thormang3_link_data_[41]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    thormang3_link_data_[42]->name_               =  "passive_pitch";
    thormang3_link_data_[42]->parent_             =  43;
    thormang3_link_data_[42]->sibling_            =  -1;
    thormang3_link_data_[42]->child_              =  41;
    thormang3_link_data_[42]->mass_               =  0.0;
    thormang3_link_data_[42]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[42]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[42]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[42]->joint_limit_max_    =  2.0 * M_PI;
    thormang3_link_data_[42]->joint_limit_min_    =  -2.0 * M_PI;
    thormang3_link_data_[42]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    thormang3_link_data_[43]->name_               =  "passive_yaw";
    thormang3_link_data_[43]->parent_             =  40;
    thormang3_link_data_[43]->sibling_            =  -1;
    thormang3_link_data_[43]->child_              =  42;
    thormang3_link_data_[43]->mass_               =  0.0;
    thormang3_link_data_[43]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[43]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[43]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[43]->joint_limit_max_    =  2.0 * M_PI;
    thormang3_link_data_[43]->joint_limit_min_    =  -2.0 * M_PI;
    thormang3_link_data_[43]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    /* ----- body -----*/

    // pelvis_link
    thormang3_link_data_[44]->name_               =  "pelvis";
    thormang3_link_data_[44]->parent_             =  41;
    thormang3_link_data_[44]->sibling_            =  -1;
    thormang3_link_data_[44]->child_              =  27;
    thormang3_link_data_[44]->mass_               =  6.869;
    thormang3_link_data_[44]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[44]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[44]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.011 , 0.000 , 0.058 );
    thormang3_link_data_[44]->joint_limit_max_    =  100.0;
    thormang3_link_data_[44]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[44]->inertia_            =  robotis_framework::getInertiaXYZ( 0.03603 , 0.00000 , 0.00016 , 0.02210 , 0.00000 , 0.03830 );

    // chest_link
    thormang3_link_data_[27]->name_               =  "torso_y";
    thormang3_link_data_[27]->parent_             =  44;
    thormang3_link_data_[27]->sibling_            =  15;
    thormang3_link_data_[27]->child_              =  28;
    thormang3_link_data_[27]->mass_               =  5.383;
    thormang3_link_data_[27]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.1705 );
    thormang3_link_data_[27]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[27]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.007 , 0.000 , 0.109 );
    thormang3_link_data_[27]->joint_limit_max_    =  0.6 * M_PI;
    thormang3_link_data_[27]->joint_limit_min_    =  -0.6 * M_PI;
    thormang3_link_data_[27]->inertia_            =  robotis_framework::getInertiaXYZ( 0.04710 , 0.00000 , 0.00036 , 0.02554 , 0.00000 , 0.03094 );

    /* ----- head -----*/

    // head_yaw
    thormang3_link_data_[28]->name_               =  "head_y";
    thormang3_link_data_[28]->parent_             =  27;
    thormang3_link_data_[28]->sibling_            =  1;
    thormang3_link_data_[28]->child_              =  29;
    thormang3_link_data_[28]->mass_               =  0.087;
    thormang3_link_data_[28]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.229 );
    thormang3_link_data_[28]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[28]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.000 , -0.002 , 0.010 );
    thormang3_link_data_[28]->joint_limit_max_    =  0.5 * M_PI;
    thormang3_link_data_[28]->joint_limit_min_    =  -0.5 * M_PI;
    thormang3_link_data_[28]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00011 , 0.00000 , 0.00000 , 0.00003 , 0.00000 , 0.00012 );

    // head_pitch
    thormang3_link_data_[29]->name_               =  "head_p";
    thormang3_link_data_[29]->parent_             =  28;
    thormang3_link_data_[29]->sibling_            =  -1;
    thormang3_link_data_[29]->child_              =  -1;
    thormang3_link_data_[29]->mass_               =  0.724;
    thormang3_link_data_[29]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , -0.04500 , 0.03900 );
    thormang3_link_data_[29]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[29]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.009 , 0.046 , 0.022 );
    thormang3_link_data_[29]->joint_limit_max_    =  0.5 * M_PI;
    thormang3_link_data_[29]->joint_limit_min_    =  -0.5 * M_PI;
    thormang3_link_data_[29]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00113 , 0.00001 , -0.00005 , 0.00114 , 0.00002 , 0.00084 );

    /*----- right arm -----*/

    // right arm shoulder pitch 1
    thormang3_link_data_[1]->name_               =  "r_arm_sh_p1";
    thormang3_link_data_[1]->parent_             =  27;
    thormang3_link_data_[1]->sibling_            =  2;
    thormang3_link_data_[1]->child_              =  3;
    thormang3_link_data_[1]->mass_               =  0.194;
    thormang3_link_data_[1]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , -0.152 , 0.160 );
    thormang3_link_data_[1]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
    thormang3_link_data_[1]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.003 , -0.020 , -0.005 );
    thormang3_link_data_[1]->joint_limit_max_    =  0.65 * M_PI;
    thormang3_link_data_[1]->joint_limit_min_    =  -0.65 * M_PI;
    thormang3_link_data_[1]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00018 , 0.0 , 0.0 , 0.00058 , -0.00004 , 0.00057 );

    // right arm shoulder roll
    thormang3_link_data_[3]->name_               =  "r_arm_sh_r";
    thormang3_link_data_[3]->parent_             =  1;
    thormang3_link_data_[3]->sibling_            =  -1;
    thormang3_link_data_[3]->child_              =  5;
    thormang3_link_data_[3]->mass_               =  0.875;
    thormang3_link_data_[3]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.057 , -0.060 , -0.039 );
    thormang3_link_data_[3]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
    thormang3_link_data_[3]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.060 , -0.002 , 0.000 );
    thormang3_link_data_[3]->joint_limit_max_    =  0.65 * M_PI;
    thormang3_link_data_[3]->joint_limit_min_    =  -0.65 * M_PI;
    thormang3_link_data_[3]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

    // right arm shoulder pitch 2
    thormang3_link_data_[5]->name_               =  "r_arm_sh_p2";
    thormang3_link_data_[5]->parent_             =  3;
    thormang3_link_data_[5]->sibling_            =  -1;
    thormang3_link_data_[5]->child_              =  7;
    thormang3_link_data_[5]->mass_               =  1.122;
    thormang3_link_data_[5]->relative_position_  =  robotis_framework::getTransitionXYZ( -0.057 , -0.033 , 0.000 );
    thormang3_link_data_[5]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
    thormang3_link_data_[5]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.000 , -0.073 , 0.000 );
    thormang3_link_data_[5]->joint_limit_max_    =  0.65 * M_PI;
    thormang3_link_data_[5]->joint_limit_min_    =  -0.65 * M_PI;
    thormang3_link_data_[5]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00277 , 0.00002 , -0.00001 , 0.00090 , 0.00004 , 0.00255 );

    // right arm elbow yaw
    thormang3_link_data_[7]->name_               =  "r_arm_el_y";
    thormang3_link_data_[7]->parent_             =  5;
    thormang3_link_data_[7]->sibling_            =  -1;
    thormang3_link_data_[7]->child_              =  9;
    thormang3_link_data_[7]->mass_               =  1.357;
    thormang3_link_data_[7]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.03000 , -0.18700 , 0.05700 );
    thormang3_link_data_[7]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
    thormang3_link_data_[7]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.042 , -0.012 , -0.058 );
    thormang3_link_data_[7]->joint_limit_max_    =  0.45 * M_PI;
    thormang3_link_data_[7]->joint_limit_min_    =  -0.45 * M_PI;
    thormang3_link_data_[7]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00152 , 0.00100 , -0.00006 , 0.00560 , 0.00002 , 0.00528 );

    // right arm wrist roll
    thormang3_link_data_[9]->name_               =  "r_arm_wr_r";
    thormang3_link_data_[9]->parent_             =  7;
    thormang3_link_data_[9]->sibling_            =  -1;
    thormang3_link_data_[9]->child_              =  11;
    thormang3_link_data_[9]->mass_               =  0.087;
    thormang3_link_data_[9]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.171 , -0.030 , -0.057 );
    thormang3_link_data_[9]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
    thormang3_link_data_[9]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.010 , 0.000 , -0.002 );
    thormang3_link_data_[9]->joint_limit_max_    =  0.9 * M_PI;
    thormang3_link_data_[9]->joint_limit_min_    =  -0.9 * M_PI;
    thormang3_link_data_[9]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00012 , 0.00000 , 0.00000 , 0.00011 , 0.00000 , 0.00003 );

    // right arm wrist yaw
    thormang3_link_data_[11]->name_               =  "r_arm_wr_y";
    thormang3_link_data_[11]->parent_             =  9;
    thormang3_link_data_[11]->sibling_            =  -1;
    thormang3_link_data_[11]->child_              =  13;
    thormang3_link_data_[11]->mass_               =  0.768;
    thormang3_link_data_[11]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.039 , 0.000 , 0.045 );
    thormang3_link_data_[11]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
    thormang3_link_data_[11]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.023 , -0.001 , -0.046 );
    thormang3_link_data_[11]->joint_limit_max_    =  0.45 * M_PI;
    thormang3_link_data_[11]->joint_limit_min_    =  -0.45 * M_PI;
    thormang3_link_data_[11]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00059 , 0.00002 , -0.00002 , 0.00078 , 0.00000 , 0.00078 );

    // right arm wrist pitch
    thormang3_link_data_[13]->name_               =  "r_arm_wr_p";
    thormang3_link_data_[13]->parent_             =  11;
    thormang3_link_data_[13]->sibling_            =  -1;
    thormang3_link_data_[13]->child_              =  31;
    thormang3_link_data_[13]->mass_               =  0.565;
    thormang3_link_data_[13]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.045 , 0.045 , -0.045 );
    thormang3_link_data_[13]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[13]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.065 , -0.045 , 0.000 );
    thormang3_link_data_[13]->joint_limit_max_    =  0.45 * M_PI;
    thormang3_link_data_[13]->joint_limit_min_    =  -0.45 * M_PI;
    thormang3_link_data_[13]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00047 , 0.00001 , 0.00000 , 0.00042 , 0.00000 , 0.00058 );

    // right arm gripper
    thormang3_link_data_[31]->name_               =  "r_arm_grip";
    thormang3_link_data_[31]->parent_             =  13;
    thormang3_link_data_[31]->sibling_            =  33;
    thormang3_link_data_[31]->child_              =  -1;
    thormang3_link_data_[31]->mass_               =  0.013;
    thormang3_link_data_[31]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.088 , -0.058 , 0.000 );
    thormang3_link_data_[31]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[31]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[31]->joint_limit_max_    =  0.5 * M_PI;
    thormang3_link_data_[31]->joint_limit_min_    =  -0.5 * M_PI;
    thormang3_link_data_[31]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    // right arm gripper 1
    thormang3_link_data_[33]->name_               =  "r_arm_grip_1";
    thormang3_link_data_[33]->parent_             =  13;
    thormang3_link_data_[33]->sibling_            =  35;
    thormang3_link_data_[33]->child_              =  -1;
    thormang3_link_data_[33]->mass_               =  0.013;
    thormang3_link_data_[33]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.088 , -0.032 , 0.000 );
    thormang3_link_data_[33]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[33]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[33]->joint_limit_max_    =  0.5 * M_PI;
    thormang3_link_data_[33]->joint_limit_min_    =  -0.5 * M_PI;
    thormang3_link_data_[33]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    // right arm end effector
    thormang3_link_data_[35]->name_               =  "r_arm_end";
    thormang3_link_data_[35]->parent_             =  13;
    thormang3_link_data_[35]->sibling_            =  -1;
    thormang3_link_data_[35]->child_              =  -1;
    thormang3_link_data_[35]->mass_               =  0.0;
    thormang3_link_data_[35]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.145 , -0.045 , 0.0 );
    thormang3_link_data_[35]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[35]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[35]->joint_limit_max_    =  100.0;
    thormang3_link_data_[35]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[35]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    /*----- left arm -----*/

    // left arm shoulder pitch 1
    thormang3_link_data_[2]->name_               =  "l_arm_sh_p1";
    thormang3_link_data_[2]->parent_             =  27;
    thormang3_link_data_[2]->sibling_            =  -1;
    thormang3_link_data_[2]->child_              =  4;
    thormang3_link_data_[2]->mass_               =  0.194;
    thormang3_link_data_[2]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , 0.152 , 0.160 );
    thormang3_link_data_[2]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[2]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.003 , 0.020 , -0.005 );
    thormang3_link_data_[2]->joint_limit_max_    =  0.65 * M_PI;
    thormang3_link_data_[2]->joint_limit_min_    =  -0.65 * M_PI;
    thormang3_link_data_[2]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00018 , 0.00000 , 0.00000 , 0.00058 , 0.00004 , 0.00057 );

    // left arm shoulder roll
    thormang3_link_data_[4]->name_               =  "l_arm_sh_r";
    thormang3_link_data_[4]->parent_             =  2;
    thormang3_link_data_[4]->sibling_            =  -1;
    thormang3_link_data_[4]->child_              =  6;
    thormang3_link_data_[4]->mass_               =  0.875;
    thormang3_link_data_[4]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.057 , 0.060 , -0.039 );
    thormang3_link_data_[4]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
    thormang3_link_data_[4]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.060 , 0.002 , 0.000 );
    thormang3_link_data_[4]->joint_limit_max_    =  0.65 * M_PI;
    thormang3_link_data_[4]->joint_limit_min_    =  -0.65 * M_PI;
    thormang3_link_data_[4]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

    // left arm shoulder pitch 2
    thormang3_link_data_[6]->name_               =  "l_arm_sh_p2";
    thormang3_link_data_[6]->parent_             =  4;
    thormang3_link_data_[6]->sibling_            =  -1;
    thormang3_link_data_[6]->child_              =  8;
    thormang3_link_data_[6]->mass_               =  1.122;
    thormang3_link_data_[6]->relative_position_  =  robotis_framework::getTransitionXYZ( -0.057 , 0.033 , 0.000 );
    thormang3_link_data_[6]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[6]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.000 , 0.073 , 0.000 );
    thormang3_link_data_[6]->joint_limit_max_    =  0.65 * M_PI;
    thormang3_link_data_[6]->joint_limit_min_    =  -0.65 * M_PI;
    thormang3_link_data_[6]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00277 , -0.00002 , -0.00001 , 0.00090 , -0.00004 , 0.00255 );

    // left arm elbow yaw
    thormang3_link_data_[8]->name_               =  "l_arm_el_y";
    thormang3_link_data_[8]->parent_             =  6;
    thormang3_link_data_[8]->sibling_            =  -1;
    thormang3_link_data_[8]->child_              =  10;
    thormang3_link_data_[8]->mass_               =  1.357;
    thormang3_link_data_[8]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.030 , 0.187 , 0.057 );
    thormang3_link_data_[8]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
    thormang3_link_data_[8]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.042 , 0.012 , -0.058 );
    thormang3_link_data_[8]->joint_limit_max_    =  0.45 * M_PI;
    thormang3_link_data_[8]->joint_limit_min_    =  -0.45 * M_PI;
    thormang3_link_data_[8]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00152 , -0.00100 , -0.00006 , 0.00560 , -0.00002 , 0.00528 );

    // left arm wrist roll
    thormang3_link_data_[10]->name_               =  "l_arm_wr_r";
    thormang3_link_data_[10]->parent_             =  8;
    thormang3_link_data_[10]->sibling_            =  -1;
    thormang3_link_data_[10]->child_              =  12;
    thormang3_link_data_[10]->mass_               =  0.087;
    thormang3_link_data_[10]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.171 , 0.030 , -0.057 );
    thormang3_link_data_[10]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
    thormang3_link_data_[10]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.010 , 0.000 , 0.002 );
    thormang3_link_data_[10]->joint_limit_max_    =  0.9 * M_PI;
    thormang3_link_data_[10]->joint_limit_min_    =  -0.9 * M_PI;
    thormang3_link_data_[10]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00012 , 0.00000 , 0.00000 , 0.00011 , 0.00000 , 0.00003 );

    // left arm wrist yaw
    thormang3_link_data_[12]->name_               =  "l_arm_wr_y";
    thormang3_link_data_[12]->parent_             =  10;
    thormang3_link_data_[12]->sibling_            =  -1;
    thormang3_link_data_[12]->child_              =  14;
    thormang3_link_data_[12]->mass_               =  0.768;
    thormang3_link_data_[12]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.039 , 0.000 , 0.045 );
    thormang3_link_data_[12]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
    thormang3_link_data_[12]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.023 , 0.001 , -0.046 );
    thormang3_link_data_[12]->joint_limit_max_    =  0.45 * M_PI;
    thormang3_link_data_[12]->joint_limit_min_    =  -0.45 * M_PI;
    thormang3_link_data_[12]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00059 , -0.00002 , -0.00002 , 0.00078 , 0.00000 , 0.00078 );

    // left arm wrist pitch
    thormang3_link_data_[14]->name_               =  "l_arm_wr_p";
    thormang3_link_data_[14]->parent_             =  12;
    thormang3_link_data_[14]->sibling_            =  -1;
    thormang3_link_data_[14]->child_              =  30;
    thormang3_link_data_[14]->mass_               =  0.565;
    thormang3_link_data_[14]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.045 , -0.045 , -0.045 );
    thormang3_link_data_[14]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
    thormang3_link_data_[14]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.065 , 0.045 , 0.000 );
    thormang3_link_data_[14]->joint_limit_max_    =  0.45 * M_PI;
    thormang3_link_data_[14]->joint_limit_min_    =  -0.45 * M_PI;
    thormang3_link_data_[14]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00047 , -0.00001 , 0.00000 , 0.00042 , 0.00000 , 0.00058 );

    // left arm gripper
    thormang3_link_data_[30]->name_               =  "l_arm_grip";
    thormang3_link_data_[30]->parent_             =  14;
    thormang3_link_data_[30]->sibling_            =  32;
    thormang3_link_data_[30]->child_              =  -1;
    thormang3_link_data_[30]->mass_               =  0.013;
    thormang3_link_data_[30]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.088 , 0.058 , 0.000 );
    thormang3_link_data_[30]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[30]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[30]->joint_limit_max_    =  0.5 * M_PI;
    thormang3_link_data_[30]->joint_limit_min_    =  -0.5 * M_PI;
    thormang3_link_data_[30]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    // left arm gripper_1
    thormang3_link_data_[32]->name_               =  "l_arm_grip_1";
    thormang3_link_data_[32]->parent_             =  14;
    thormang3_link_data_[32]->sibling_            =  34;
    thormang3_link_data_[32]->child_              =  -1;
    thormang3_link_data_[32]->mass_               =  0.013;
    thormang3_link_data_[32]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.088 , 0.032 , 0.000 );
    thormang3_link_data_[32]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
    thormang3_link_data_[32]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[32]->joint_limit_max_    =  0.5 * M_PI;
    thormang3_link_data_[32]->joint_limit_min_    =  -0.5 * M_PI;
    thormang3_link_data_[32]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    // left arm end effector
    thormang3_link_data_[34]->name_               =  "l_arm_end";
    thormang3_link_data_[34]->parent_             =  14;
    thormang3_link_data_[34]->sibling_            =  -1;
    thormang3_link_data_[34]->child_              =  -1;
    thormang3_link_data_[34]->mass_               =  0.0;
    thormang3_link_data_[34]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.145 , 0.045 , 0.0 );
    thormang3_link_data_[34]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[34]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[34]->joint_limit_max_    =  100.0;
    thormang3_link_data_[34]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[34]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    /* ----- right leg -----*/

    // right leg hip yaw
    thormang3_link_data_[15]->name_               =  "r_leg_hip_y";
    thormang3_link_data_[15]->parent_             =  44;
    thormang3_link_data_[15]->sibling_            =  16;
    thormang3_link_data_[15]->child_              =  17;
    thormang3_link_data_[15]->mass_               =  0.243;
    thormang3_link_data_[15]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , -0.093 , -0.018 );
    thormang3_link_data_[15]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
    thormang3_link_data_[15]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.012 , 0.000 , -0.025 );
    thormang3_link_data_[15]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
    thormang3_link_data_[15]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
    thormang3_link_data_[15]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00024 , 0.00000 , 0.00000 , 0.00101 , 0.00000 , 0.00092 );

    // right leg hip roll
    thormang3_link_data_[17]->name_               =  "r_leg_hip_r";
    thormang3_link_data_[17]->parent_             =  15;
    thormang3_link_data_[17]->sibling_            =  -1;
    thormang3_link_data_[17]->child_              =  19;
    thormang3_link_data_[17]->mass_               =  1.045;
    thormang3_link_data_[17]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.057 , 0.000 , -0.075 );
    thormang3_link_data_[17]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
    thormang3_link_data_[17]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.068 , 0.000 , 0.000 );
    thormang3_link_data_[17]->joint_limit_max_    =  2.0 * M_PI; //0.3 * M_PI;
    thormang3_link_data_[17]->joint_limit_min_    =  -2.0 * M_PI; //-0.3 * M_PI;
    thormang3_link_data_[17]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

    // right leg hip pitch
    thormang3_link_data_[19]->name_               =  "r_leg_hip_p";
    thormang3_link_data_[19]->parent_             =  17;
    thormang3_link_data_[19]->sibling_            =  -1;
    thormang3_link_data_[19]->child_              =  21;
    thormang3_link_data_[19]->mass_               =  3.095;
    thormang3_link_data_[19]->relative_position_  =  robotis_framework::getTransitionXYZ( -0.057 , -0.033 , 0.000 );
    thormang3_link_data_[19]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
    thormang3_link_data_[19]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.022 , 0.007 , -0.168 );
    thormang3_link_data_[19]->joint_limit_max_    =  2.0 * M_PI; //0.4 * M_PI;
    thormang3_link_data_[19]->joint_limit_min_    =  -2.0 * M_PI; //-0.4 * M_PI;
    thormang3_link_data_[19]->inertia_            =  robotis_framework::getInertiaXYZ( 0.04329 , -0.00027 , 0.00286 , 0.04042 , 0.00203 , 0.00560 );

    // right leg knee pitch
    thormang3_link_data_[21]->name_               =  "r_leg_kn_p";
    thormang3_link_data_[21]->parent_             =  19;
    thormang3_link_data_[21]->sibling_            =  -1;
    thormang3_link_data_[21]->child_              =  23;
    thormang3_link_data_[21]->mass_               =  2.401;
    thormang3_link_data_[21]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , -0.060 , -0.300 );
    thormang3_link_data_[21]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
    thormang3_link_data_[21]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.002 , 0.066 , -0.183 );
    thormang3_link_data_[21]->joint_limit_max_    =  2.0 * M_PI; //0.1 * M_PI;
    thormang3_link_data_[21]->joint_limit_min_    =  -2.0 * M_PI; //-0.7 * M_PI;
    thormang3_link_data_[21]->inertia_            =  robotis_framework::getInertiaXYZ( 0.01971 , -0.00031 , -0.00294 , 0.01687 , -0.00140 , 0.00574 );

    // right leg ankle pitch
    thormang3_link_data_[23]->name_               =  "r_leg_an_p";
    thormang3_link_data_[23]->parent_             =  21;
    thormang3_link_data_[23]->sibling_            =  -1;
    thormang3_link_data_[23]->child_              =  25;
    thormang3_link_data_[23]->mass_               =  1.045;
    thormang3_link_data_[23]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , 0.060 , -0.300 );
    thormang3_link_data_[23]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[23]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.011 , 0.033 , 0.000 );
    thormang3_link_data_[23]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
    thormang3_link_data_[23]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
    thormang3_link_data_[23]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

    // right leg ankle roll
    thormang3_link_data_[25]->name_               =  "r_leg_an_r";
    thormang3_link_data_[25]->parent_             =  23;
    thormang3_link_data_[25]->sibling_            =  -1;
    thormang3_link_data_[25]->child_              =  37;
    thormang3_link_data_[25]->mass_               =  0.223;
    thormang3_link_data_[25]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.057 , 0.033 , 0.000 );
    thormang3_link_data_[25]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
    thormang3_link_data_[25]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.070 , 0.000 , -0.048 );
    thormang3_link_data_[25]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
    thormang3_link_data_[25]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
    thormang3_link_data_[25]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00022 , 0.00000 , -0.00001 , 0.00099 , 0.00000 , 0.00091 );

    // right leg ft
    thormang3_link_data_[37]->name_               =  "r_leg_ft";
    thormang3_link_data_[37]->parent_             =  25;
    thormang3_link_data_[37]->sibling_            =  -1;
    thormang3_link_data_[37]->child_              =  45;
    thormang3_link_data_[37]->mass_               =  1.689;
    thormang3_link_data_[37]->relative_position_  =  robotis_framework::getTransitionXYZ( -0.057 , 0.000 , -0.087 );
    thormang3_link_data_[37]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[37]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.000 , -0.009 , -0.013 );
    thormang3_link_data_[37]->joint_limit_max_    =  100.0;
    thormang3_link_data_[37]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[37]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00219 , 0.00000 , 0.00000 , 0.00433 , -0.00011 , 0.00609 );

    // right leg end
    thormang3_link_data_[45]->name_               =  "r_leg_end";
    thormang3_link_data_[45]->parent_             =  37;
    thormang3_link_data_[45]->sibling_            =  -1;
    thormang3_link_data_[45]->child_              =  -1;
    thormang3_link_data_[45]->mass_               =  0.0;
    thormang3_link_data_[45]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -0.0275 );
    thormang3_link_data_[45]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[45]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[45]->joint_limit_max_    =  100.0;
    thormang3_link_data_[45]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[45]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    /* ----- left leg -----*/

    // left leg hip yaw
    thormang3_link_data_[16]->name_               =  "l_leg_hip_y";
    thormang3_link_data_[16]->parent_             =  44;
    thormang3_link_data_[16]->sibling_            =  -1;
    thormang3_link_data_[16]->child_              =  18;
    thormang3_link_data_[16]->mass_               =  0.243;
    thormang3_link_data_[16]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , 0.093 , -0.018 );
    thormang3_link_data_[16]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -1.0 );
    thormang3_link_data_[16]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.012 , 0.000 , -0.025 );
    thormang3_link_data_[16]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
    thormang3_link_data_[16]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
    thormang3_link_data_[16]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00024 , 0.00000 , 0.00000 , 0.00101 , 0.00000 , 0.00092 );

    // left leg hip roll
    thormang3_link_data_[18]->name_               =  "l_leg_hip_r";
    thormang3_link_data_[18]->parent_             =  16;
    thormang3_link_data_[18]->sibling_            =  -1;
    thormang3_link_data_[18]->child_              =  20;
    thormang3_link_data_[18]->mass_               =  1.045;
    thormang3_link_data_[18]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.057 , 0.000 , -0.075 );
    thormang3_link_data_[18]->joint_axis_         =  robotis_framework::getTransitionXYZ( -1.0 , 0.0 , 0.0 );
    thormang3_link_data_[18]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.068 , 0.000 , 0.000 );
    thormang3_link_data_[18]->joint_limit_max_    =  2.0 * M_PI; //0.3 * M_PI;
    thormang3_link_data_[18]->joint_limit_min_    =  -2.0 * M_PI; //-0.3 * M_PI;
    thormang3_link_data_[18]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

    // left leg hip pitch
    thormang3_link_data_[20]->name_               =  "l_leg_hip_p";
    thormang3_link_data_[20]->parent_             =  18;
    thormang3_link_data_[20]->sibling_            =  -1;
    thormang3_link_data_[20]->child_              =  22;
    thormang3_link_data_[20]->mass_               =  3.095;
    thormang3_link_data_[20]->relative_position_  =  robotis_framework::getTransitionXYZ( -0.057 , 0.033 , 0.000 );
    thormang3_link_data_[20]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[20]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.022 , -0.007 , -0.168 );
    thormang3_link_data_[20]->joint_limit_max_    =  2.0 * M_PI; //0.4 * M_PI;
    thormang3_link_data_[20]->joint_limit_min_    =  -2.0 * M_PI; //-0.4 * M_PI;
    thormang3_link_data_[20]->inertia_            =  robotis_framework::getInertiaXYZ( 0.04328 , 0.00028 , 0.00288 , 0.04042 , -0.00202 , 0.00560 );

    // left leg knee pitch
    thormang3_link_data_[22]->name_               =  "l_leg_kn_p";
    thormang3_link_data_[22]->parent_             =  20;
    thormang3_link_data_[22]->sibling_            =  -1;
    thormang3_link_data_[22]->child_              =  24;
    thormang3_link_data_[22]->mass_               =  2.401;
    thormang3_link_data_[22]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , 0.060 , -0.300 );
    thormang3_link_data_[22]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
    thormang3_link_data_[22]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.002 , -0.066 , -0.183 );
    thormang3_link_data_[22]->joint_limit_max_    =  2.0 * M_PI; //0.7 * M_PI;
    thormang3_link_data_[22]->joint_limit_min_    =  -2.0 * M_PI; //-0.1 * M_PI;
    thormang3_link_data_[22]->inertia_            =  robotis_framework::getInertiaXYZ( 0.01971 , 0.00031 , -0.00294 , 0.01687 , 0.00140 , 0.00574 );

    // left leg ankle pitch
    thormang3_link_data_[24]->name_               =  "l_leg_an_p";
    thormang3_link_data_[24]->parent_             =  22;
    thormang3_link_data_[24]->sibling_            =  -1;
    thormang3_link_data_[24]->child_              =  26;
    thormang3_link_data_[24]->mass_               =  1.045;
    thormang3_link_data_[24]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.000 , -0.060 , -0.300 );
    thormang3_link_data_[24]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , -1.0 , 0.0 );
    thormang3_link_data_[24]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.011 , -0.033 , 0.000 );
    thormang3_link_data_[24]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
    thormang3_link_data_[24]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
    thormang3_link_data_[24]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

    // left leg ankle pitch
    thormang3_link_data_[26]->name_               =  "l_leg_an_r";
    thormang3_link_data_[26]->parent_             =  24;
    thormang3_link_data_[26]->sibling_            =  -1;
    thormang3_link_data_[26]->child_              =  36;
    thormang3_link_data_[26]->mass_               =  0.223;
    thormang3_link_data_[26]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.057 , -0.033 , 0.000 );
    thormang3_link_data_[26]->joint_axis_         =  robotis_framework::getTransitionXYZ( 1.0 , 0.0 , 0.0 );
    thormang3_link_data_[26]->center_of_mass_     =  robotis_framework::getTransitionXYZ( -0.070 , 0.000 , -0.048 );
    thormang3_link_data_[26]->joint_limit_max_    =  2.0 * M_PI; //0.45 * M_PI;
    thormang3_link_data_[26]->joint_limit_min_    =  -2.0 * M_PI; //-0.45 * M_PI;
    thormang3_link_data_[26]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00022 , 0.00000 , -0.00001 , 0.00099 , 0.00000 , 0.00091 );

    // left leg ft
    thormang3_link_data_[36]->name_               =  "l_leg_ft";
    thormang3_link_data_[36]->parent_             =  26;
    thormang3_link_data_[36]->sibling_            =  -1;
    thormang3_link_data_[36]->child_              =  46;
    thormang3_link_data_[36]->mass_               =  1.689;
    thormang3_link_data_[36]->relative_position_  =  robotis_framework::getTransitionXYZ( -0.057 , 0.000 , -0.087 );
    thormang3_link_data_[36]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[36]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.000 , 0.009 , -0.013 );
    thormang3_link_data_[36]->joint_limit_max_    =  100.0;
    thormang3_link_data_[36]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[36]->inertia_            =  robotis_framework::getInertiaXYZ( 0.00219 , 0.00000 , 0.00000 , 0.00433 , 0.00011 , 0.00609 );

    // left leg end
    thormang3_link_data_[46]->name_               =  "l_leg_end";
    thormang3_link_data_[46]->parent_             =  36;
    thormang3_link_data_[46]->sibling_            =  -1;
    thormang3_link_data_[46]->child_              =  -1;
    thormang3_link_data_[46]->mass_               =  0.0;
    thormang3_link_data_[46]->relative_position_  =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , -0.0275 );
    thormang3_link_data_[46]->joint_axis_         =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[46]->center_of_mass_     =  robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    thormang3_link_data_[46]->joint_limit_max_    =  100.0;
    thormang3_link_data_[46]->joint_limit_min_    =  -100.0;
    thormang3_link_data_[46]->inertia_            =  robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );
  }

  thigh_length_m_ = std::fabs(thormang3_link_data_[ID_R_LEG_START+2*3]->relative_position_.coeff(2,0));
  calf_length_m_  = std::fabs(thormang3_link_data_[ID_R_LEG_START+2*4]->relative_position_.coeff(2,0));
  ankle_length_m_ = std::fabs(thormang3_link_data_[ID_R_LEG_FT]->relative_position_.coeff(2,0)
                              + thormang3_link_data_[ID_R_LEG_END]->relative_position_.coeff(2,0));
  leg_side_offset_m_ 	= 2.0*(std::fabs(thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)));

  for(int joint_idx = 0; joint_idx < ALL_JOINT_ID; joint_idx++)
  {
    thormang3_link_data_[joint_idx]->joint_center_of_mass_ = thormang3_link_data_[joint_idx]->center_of_mass_;
  }
}

std::vector<int> KinematicsDynamics::findRoute(int to)
{
  int id = thormang3_link_data_[to]->parent_;

  std::vector<int> idx;

  if(id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> KinematicsDynamics::findRoute(int from, int to)
{
  int id = thormang3_link_data_[to]->parent_;

  std::vector<int> idx;

  if(id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double KinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = thormang3_link_data_[joint_id]->mass_ + calcTotalMass(thormang3_link_data_[ joint_id ]->sibling_) + calcTotalMass(thormang3_link_data_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd KinematicsDynamics::calcMassCenter(int joint_id)
{
  Eigen::MatrixXd mc(3,1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3,1);
  else
  {
    mc = thormang3_link_data_[ joint_id ]->mass_ * ( thormang3_link_data_[ joint_id ]->orientation_ * thormang3_link_data_[ joint_id ]->center_of_mass_ + thormang3_link_data_[ joint_id ]->position_ );
    mc = mc + calcMassCenter( thormang3_link_data_[ joint_id ]->sibling_ ) + calcMassCenter( thormang3_link_data_[ joint_id ]->child_ );
  }

  return mc;
}

void KinematicsDynamics::calcJointsCenterOfMass(int joint_id)
{
  if(joint_id != -1)
  {
    LinkData *temp_data = thormang3_link_data_[ joint_id ];
    temp_data->joint_center_of_mass_
      = ( temp_data->orientation_ * temp_data->center_of_mass_ + temp_data->position_ );

    calcJointsCenterOfMass(temp_data->sibling_);
    calcJointsCenterOfMass(temp_data->child_);
  }
  else
    return;
}

Eigen::MatrixXd KinematicsDynamics::calcCenterOfMass(Eigen::MatrixXd mc)
{
  double mass ;
  Eigen::MatrixXd COM(3,1);

  mass = calcTotalMass(0);
  COM = mc/mass;

  return COM;
}

void KinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    thormang3_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3,1);
    thormang3_link_data_[0]->orientation_ =
        robotis_framework::calcRodrigues( robotis_framework::calcHatto( thormang3_link_data_[0]->joint_axis_ ), thormang3_link_data_[ 0 ]->joint_angle_ );
  }

  if ( joint_id != 0 )
  {
    int parent = thormang3_link_data_[joint_id]->parent_;

    thormang3_link_data_[joint_id]->position_ =
        thormang3_link_data_[parent]->orientation_ * thormang3_link_data_[joint_id]->relative_position_ + thormang3_link_data_[parent]->position_;
    thormang3_link_data_[ joint_id ]->orientation_ =
        thormang3_link_data_[ parent ]->orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(thormang3_link_data_[joint_id]->joint_axis_), thormang3_link_data_[joint_id]->joint_angle_);

    //    thormang3_link_data_[joint_id]->transformation_.block<3,1>(0,3) = thormang3_link_data_[joint_id]->position_;
    //    thormang3_link_data_[joint_id]->transformation_.block<3,3>(0,0) = thormang3_link_data_[joint_id]->orientation_;
  }

  calcForwardKinematics(thormang3_link_data_[joint_id]->sibling_);
  calcForwardKinematics(thormang3_link_data_[joint_id]->child_);
}

Eigen::MatrixXd KinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size-1;

  Eigen::MatrixXd tar_position = thormang3_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,idx_size);

  for (int id=0; id<idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation = thormang3_link_data_[curr_id]->orientation_ * thormang3_link_data_[curr_id]->joint_axis_;

    jacobian.block(0,id,3,1) = robotis_framework::calcCross(tar_orientation,tar_position-thormang3_link_data_[curr_id]->position_);
    jacobian.block(3,id,3,1) = tar_orientation;
  }

  return jacobian;
}

Eigen::MatrixXd KinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size-1;

  Eigen::MatrixXd tar_position = thormang3_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(6,idx_size);

  for (int id=0; id<idx_size; id++)
  {
    int curr_id = idx[id];
    double mass = calcTotalMass(curr_id);

    Eigen::MatrixXd og = calcMassCenter(curr_id)/mass-thormang3_link_data_[curr_id]->position_;
    Eigen::MatrixXd tar_orientation = thormang3_link_data_[curr_id]->orientation_*thormang3_link_data_[curr_id]->joint_axis_;

    jacobian_com.block(0,id,3,1) = robotis_framework::calcCross(tar_orientation,og);
    jacobian_com.block(3,id,3,1) = tar_orientation;
  }

  return jacobian_com;
}

Eigen::MatrixXd KinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err = curr_orientation.transpose() * tar_orientation;
  Eigen::MatrixXd ori_err_dash = curr_orientation * robotis_framework::convertRotToOmega(ori_err);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6,1);
  err.block<3,1>(0,0) = pos_err;
  err.block<3,1>(3,0) = ori_err_dash;

  return err;
}

bool KinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = thormang3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = thormang3_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inverse = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inverse * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_num = idx[id];
      thormang3_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      = 	idx[ id ];

    if ( thormang3_link_data_[ joint_num ]->joint_angle_ >= thormang3_link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( thormang3_link_data_[ joint_num ]->joint_angle_ <= thormang3_link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = thormang3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = thormang3_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_num = idx[id];
      thormang3_link_data_[joint_num]->joint_angle_ +=delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      =   idx[ id ];

    if ( thormang3_link_data_[ joint_num ]->joint_angle_ >= thormang3_link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( thormang3_link_data_[ joint_num ]->joint_angle_ <= thormang3_link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(), idx.size());

  for ( int ix = 0; ix < idx.size(); ix++ )
    weight_matrix.coeffRef(ix,ix) = weight.coeff(idx[ix],0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6,6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix=0; ix<3; ix++)
  {
    eval.coeffRef(ix,ix) = p_damping;
    eval.coeffRef(ix+3,ix+3) = R_damping;
  }

  /* ik */
  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = thormang3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = thormang3_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_id = idx[id];
      thormang3_link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  /* check joint limit */
  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      =   idx[ id ];

    if ( thormang3_link_data_[ joint_num ]->joint_angle_ >= thormang3_link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( thormang3_link_data_[ joint_num ]->joint_angle_ <= thormang3_link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err, Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(),idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);
    Eigen::MatrixXd curr_position = thormang3_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = thormang3_link_data_[to]->orientation_;
    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();
    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
      thormang3_link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }
    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int _joint_num = idx[id];
    if (thormang3_link_data_[_joint_num]->joint_angle_ >= thormang3_link_data_[_joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (thormang3_link_data_[_joint_num]->joint_angle_ <= thormang3_link_data_[_joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  //Eigen::MatrixXd target_transform;
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc, trans_ac;
  Eigen::Vector3d vec;

  bool  invertible;
  double rac, arc_cos, arc_tan, k, l, m, n, s, c, theta;
  double thigh_length = thigh_length_m_;
  double calf_length = calf_length_m_;
  double ankle_length = ankle_length_m_;

  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, roll, pitch, yaw);

  vec.coeffRef(0) = trans_ad.coeff(0,3) + trans_ad.coeff(0,2) * ankle_length;
  vec.coeffRef(1) = trans_ad.coeff(1,3) + trans_ad.coeff(1,2) * ankle_length;
  vec.coeffRef(2) = trans_ad.coeff(2,3) + trans_ad.coeff(2,2) * ankle_length;

  // Get Knee
  rac = vec.norm();
  arc_cos = acos((rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
  if(std::isnan(arc_cos) == 1)
    return false;
  *(out + 3) = arc_cos;

  // Get Ankle Roll
  trans_ad.computeInverseWithCheck(trans_da, invertible);
  if(invertible == false)
    return false;

  k = sqrt(trans_da.coeff(1,3) * trans_da.coeff(1,3) +  trans_da.coeff(2,3) * trans_da.coeff(2,3));
  l = sqrt(trans_da.coeff(1,3) * trans_da.coeff(1,3) + (trans_da.coeff(2,3) - ankle_length)*(trans_da.coeff(2,3) - ankle_length));
  m = (k * k - l * l - ankle_length * ankle_length) / (2.0 * l * ankle_length);

  if(m > 1.0)
    m = 1.0;
  else if(m < -1.0)
    m = -1.0;
  arc_cos = acos(m);

  if(std::isnan(arc_cos) == 1)
    return false;

  if(trans_da.coeff(1,3) < 0.0)
    *(out + 5) = -arc_cos;
  else
    *(out + 5) = arc_cos;

  // Get Hip Yaw
  trans_cd = robotis_framework::getTransformationXYZRPY(0, 0, -ankle_length, *(out + 5), 0, 0);
  trans_cd.computeInverseWithCheck(trans_dc, invertible);
  if(invertible == false)
    return false;

  trans_ac = trans_ad * trans_dc;
  arc_tan = atan2(-trans_ac.coeff(0,1) , trans_ac.coeff(1,1));
  if(std::isinf(arc_tan) != 0)
    return false;
  *(out) = arc_tan;

  // Get Hip Roll
  arc_tan = atan2(trans_ac.coeff(2,1), -trans_ac.coeff(0,1) * sin(*(out)) + trans_ac.coeff(1,1) * cos(*(out)));
  if(std::isinf(arc_tan) != 0)
    return false;
  *(out + 1) = arc_tan;

  // Get Hip Pitch and Ankle Pitch
  arc_tan = atan2(trans_ac.coeff(0,2) * cos(*(out)) + trans_ac.coeff(1,2) * sin(*(out)), trans_ac.coeff(0,0) * cos(*(out)) + trans_ac.coeff(1,0) * sin(*(out)));
  if(std::isinf(arc_tan) == 1)
    return false;
  theta = arc_tan;
  k = sin(*(out + 3)) * calf_length;
  l = -thigh_length - cos(*(out + 3)) * calf_length;
  m = cos(*(out)) * vec.coeff(0) + sin(*(out)) * vec.coeff(1);
  n = cos(*(out + 1)) * vec.coeff(2) + sin(*(out)) * sin(*(out + 1)) * vec.coeff(0) - cos(*(out)) * sin(*(out + 1)) * vec.coeff(1);
  s = (k * n + l * m) / (k * k + l * l);
  c = (n - k * s) / l;
  arc_tan = atan2(s, c);
  if(std::isinf(arc_tan) == 1)
    return false;
  *(out + 2) = arc_tan;
  *(out + 4) = theta - *(out + 3) - *(out + 2);

  return true;
}

bool KinematicsDynamics::calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    *(out + 0) = out[0] * (thormang3_link_data_[ID_R_LEG_START + 2*0]->joint_axis_.coeff(2,0));
    *(out + 1) = out[1] * (thormang3_link_data_[ID_R_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    *(out + 2) = out[2] * (thormang3_link_data_[ID_R_LEG_START + 2*2]->joint_axis_.coeff(1,0));
    *(out + 3) = out[3] * (thormang3_link_data_[ID_R_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    *(out + 4) = out[4] * (thormang3_link_data_[ID_R_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    *(out + 5) = out[5] * (thormang3_link_data_[ID_R_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    out[0] = out[0] * (thormang3_link_data_[ID_L_LEG_START + 2*0]->joint_axis_.coeff(2,0));
    out[1] = out[1] * (thormang3_link_data_[ID_L_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    out[2] = out[2] * (thormang3_link_data_[ID_L_LEG_START + 2*2]->joint_axis_.coeff(1,0));
    out[3] = out[3] * (thormang3_link_data_[ID_L_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    out[4] = out[4] * (thormang3_link_data_[ID_L_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    out[5] = out[5] * (thormang3_link_data_[ID_L_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

}
