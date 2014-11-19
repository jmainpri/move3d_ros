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
    update_robot_ = false;
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

    // cout << "Update Move3D configuration : " << new_arm_config.transpose() << endl;

    if( q_cur_.get() == NULL )
    {
        ROS_ERROR("Current robot configuration not initialized");
        return;
    }

    q_cur_->setFromEigenVector( new_arm_config, dof_ids_ );

    if( update_robot_ )
    {
        robot_->setAndUpdate(*q_cur_);

        // OPENGL DRAW
        g3d_draw_allwin_active();
    }

    // Reset watchdog timer
    // arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);
}

void Move3DRosGui::playElementaryMotion(const std::vector<double>& current_config, const std::vector<double>& target_config)
{
    cout << __PRETTY_FUNCTION__ << endl;

    pr2_controllers_msgs::JointTrajectoryGoal command;

    double execution_timestep = 5.0;

    // Populate command
    command.trajectory.joint_names = joint_names_;
    command.trajectory.header.stamp = ros::Time::now();

    // Populate target point
    trajectory_msgs::JointTrajectoryPoint start_point;
    start_point.positions = current_config;
    start_point.velocities.resize(start_point.positions.size(), 0.0);
    start_point.time_from_start = ros::Duration(0.0);

    // Populate target point
    trajectory_msgs::JointTrajectoryPoint target_point;
    target_point.positions = target_config;
    target_point.velocities.resize(target_point.positions.size(), 0.0);
    // Set the execution time
    target_point.time_from_start = ros::Duration(execution_timestep);

    // Add point
    command.trajectory.points.push_back(target_point);

    // Command the arm
    arm_client_->sendGoal(command);
}

void Move3DRosGui::initPr2()
{
    cout << __PRETTY_FUNCTION__ << endl;

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

    q_cur_ = robot_->getInitPos();
}

void Move3DRosGui::run()
{
    cout << __PRETTY_FUNCTION__ << endl;

    robot_ = Move3D::global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
    if( robot_ == NULL ){
        ROS_ERROR("No ROBOT in Move3D");
        return;
    }

    if( robot_->getName().find("PR2") != std::string::npos )
        initPr2();
    else {
        ROS_ERROR("No robot named PR2 in Move3D");
        return;
    }

    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "move3d_pr2");
    nh_ = new ros::NodeHandle();

    // Subscribe to get current posture
    ros::Subscriber sub = nh_->subscribe("/r_arm_controller/state", 1, &Move3DRosGui::GetJointState, this);

    // Setup trajectory controller interface
    arm_client_ = MOVE3D_PTR_NAMESPACE::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> >(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(std::string("right_arm"), true));
    ROS_INFO("Waiting for arm controllers to come up...");
    arm_client_->waitForServer();

    // Spin node
    ros::Rate spin_rate(joint_state_rate_);
    ros::spin();
}

void Move3DRosGui::start()
{
    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosGui::run, this ) );
    emit(selectedPlanner(QString("BoostFunction")));
}
