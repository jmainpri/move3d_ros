/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */

#include "move3d_ros_gui.hpp"
#include "ui_move3d_ros_gui.h"

#include <std_msgs/String.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "planner_handler.hpp"

#include <libmove3d/include/Graphic-pkg.h>

#include <boost/bind.hpp>
#include <cmath>
#include <time.h>
#include <sstream>

using std::cout;
using std::endl;

Move3DRosGui::Move3DRosGui(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::Move3DRosGui)
{
    ui_->setupUi(this);
    connect(ui_->pushButtonStart,SIGNAL(clicked()),this,SLOT(start()));
    connect(this,  SIGNAL(selectedPlanner(QString)),global_plannerHandler, SLOT(startPlanner(QString)));
    joint_state_rate_ = 30;
}

Move3DRosGui::~Move3DRosGui()
{
    delete ui_;
}

void Move3DRosGui::GetJointState(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr joint_config)
{
     //cout << __PRETTY_FUNCTION__ << endl;

    if( joint_names_.empty() ){
        ROS_ERROR("Joint names not set");
        return;
    }

    // Extract joint positions in the right order
    if (joint_config->joint_names.size() != joint_config->actual.positions.size() ||
            joint_config->joint_names.size() != joint_names_.size() )
    {
        ROS_ERROR("Malformed configuration update - skipping update");
        return;
    }

    if( robot_ == NULL ){
        ROS_ERROR("No Move3D robot selected");
        return;
    }

    // Set the config
    Eigen::VectorXd new_arm_config(joint_names_.size());

    try
    {
        std::map<std::string, double> arm_configuration;
        for (size_t idx = 0; idx < joint_config->joint_names.size(); idx ++)
            arm_configuration[joint_config->joint_names[idx]] = joint_config->actual.positions[idx];

        for (size_t idx = 0; idx < new_arm_config.size(); idx ++)
            new_arm_config[idx] = arm_configuration[joint_names_[idx]];
    }
    catch(...)
    {
        ROS_ERROR("Could not map joint correctly");
        return;
    }

    //    current_arm_config_ = new_arm_config;

    cout << "Update Move3D configuration : " << new_arm_config.transpose() << endl;

    Move3D::confPtr_t q_cur = robot_->getCurrentPos();
    q_cur->setFromEigenVector( new_arm_config, dof_ids_ );
    robot_->setAndUpdate(*q_cur);

    // OPENGL DRAW
    g3d_draw_allwin_active();

    // Reset watchdog timer
    // arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);
}

void Move3DRosGui::initPr2()
{
    cout << __PRETTY_FUNCTION__ << endl;

    robot_ = Move3D::global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
    if( robot_ == NULL ){
        ROS_ERROR("No robot named pr2 in Move3D");
        return;
    }

    joint_names_.resize(7);
    joint_names_[0] = "r_shoulder_pan_joint";
    joint_names_[1] = "r_shoulder_lift_joint";
    joint_names_[2] = "r_upper_arm_roll_joint";
    joint_names_[3] = "r_elbow_flex_joint";
    joint_names_[4] = "r_forearm_roll_joint";
    joint_names_[5] = "r_wrist_flex_joint";
    joint_names_[6] = "r_wrist_roll_joint";

    dof_ids_.resize(7);
    dof_ids_[0] = robot_->getJoint("right-Arm1")->getIndexOfFirstDof();
    dof_ids_[1] = robot_->getJoint("right-Arm2")->getIndexOfFirstDof();
    dof_ids_[2] = robot_->getJoint("right-Arm3")->getIndexOfFirstDof();
    dof_ids_[3] = robot_->getJoint("right-Arm4")->getIndexOfFirstDof();
    dof_ids_[4] = robot_->getJoint("right-Arm5")->getIndexOfFirstDof();
    dof_ids_[5] = robot_->getJoint("right-Arm6")->getIndexOfFirstDof();
    dof_ids_[6] = robot_->getJoint("right-Arm7")->getIndexOfFirstDof();
}

void Move3DRosGui::init()
{
     cout << __PRETTY_FUNCTION__ << endl;

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "move3d_pr2");
    nh_ = new ros::NodeHandle();
    ros::Subscriber sub = nh_->subscribe("/r_arm_controller/state", 1, &Move3DRosGui::GetJointState, this);
    ros::Rate spin_rate(joint_state_rate_);
    ros::spin();
}

void Move3DRosGui::start()
{
    initPr2();
    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosGui::init, this ) );
    emit(selectedPlanner(QString("BoostFunction")));
}
