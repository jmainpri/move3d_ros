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

#include "move3d_ros_human.hpp"


#include <std_msgs/String.h>

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "planner_handler.hpp"
#include "planner/plannerSequences.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <time.h>
#include <sstream>

using std::cout;
using std::endl;

Move3DRosHuman::Move3DRosHuman(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::Move3DRosHuman)
{
    ui_->setupUi(this);

    // Set robot structure to 0
    robot_ = NULL;
    move3d_trajs_.clear();

    // Set all active joint ids and names to 0
    active_joint_names_.clear();
    active_dof_ids_.clear();

    joint_state_rate_ = 30;
    joint_state_received_ = 0;
    draw_rate_ = 10; // draws only the 10th time

    update_robot_ = false; // updates from sensor reading
}

Move3DRosHuman::~Move3DRosHuman()
{
    delete ui_;
}

void Move3DRosHuman::initHuman()
{
    cout << __PRETTY_FUNCTION__ << endl;

    if( robot_ == NULL ){
        ROS_ERROR("No robot named pr2 in Move3D");
        return;
    }

    robot_ = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
    q_cur_ = robot_->getInitPos();
}

void Move3DRosHuman::GetJointState(sensor_msgs::JointState::ConstPtr joint_config )
{
    //    cout << __PRETTY_FUNCTION__ << endl;

    if( joint_names.empty() ){
        ROS_ERROR("Active joint names not set");
        return;
    }

    // Extract joint positions in the right order
    if (    joint_config->joint_names.size() != joint_config->actual.positions.size() ||
            joint_config->joint_names.size() != joint_names.size() ||
            joint_config->joint_names.size() != dof_ids.size() )
    {
        ROS_ERROR("Malformed configuration update - skipping update");
        return;
    }

    if( robot_ == NULL ){
        ROS_ERROR("No Move3D robot selected");
        return;
    }

    // Set the config
    Eigen::VectorXd new_arm_config( joint_names.size() );

    try
    {
        std::map<std::string, double> arm_configuration;
        for (size_t idx = 0; idx < joint_config->joint_names.size(); idx ++)
            arm_configuration[joint_config->joint_names[idx]] = joint_config->actual.positions[idx];

        for (size_t idx = 0; idx < new_arm_config.size(); idx ++)
            new_arm_config[idx] = arm_configuration[joint_names[idx]];
    }
    catch(...)
    {
        ROS_ERROR("Could not map joint correctly");
        return;
    }

    if( q_cur_.get() == NULL )
    {
        ROS_ERROR("Current robot configuration not initialized");
        return;
    }

    q_cur_->setFromEigenVector( new_arm_config, dof_ids );

    if( update_robot_ )
    {
        robot_->setAndUpdate(*q_cur_); // This might called concurently for right and left arm (which is ok but not checked)

        // OPENGL DRAW
        if( (++joint_state_received_) % draw_rate_ == 0 ) // 0 modulo k = 0
        {
            emit(drawAllWinActive());
            joint_state_received_ = 0;
        }
    }

    // Reset watchdog timer
    // arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);
}

void Move3DRosHuman::setState(module_state_t state)
{
    if( state == online )
    {
        ui_->labelOnline->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:18pt; color:#ff0000;\">ONLINE</span></p></body></html>",
                                                          0, QApplication::UnicodeUTF8));
    }
    else
    {
        ui_->labelOnline->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:18pt; color:#0cff00;\">OFFLINE</span></p></body></html>",
                                                          0, QApplication::UnicodeUTF8));
    }
}

void Move3DRosHuman::setActiveArm(arm_t arm)
{
    // SET THE ACTIVE ARM
    arm_ = arm;

    if( arm_ == left )
    {
        active_joint_names_ = left_arm_joint_names_;
        active_dof_ids_     = left_arm_dof_ids_;
        active_state_topic_name_ = left_arm_topic_name_;
        active_arm_client_ = left_arm_client_;
    }
    else
    {
        active_joint_names_ = right_arm_joint_names_;
        active_dof_ids_     = right_arm_dof_ids_;
        active_state_topic_name_ = right_arm_topic_name_;
        active_arm_client_ = right_arm_client_;
    }
}

void Move3DRosHuman::run()
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
    ros::init( argc, argv, "move3d_pr2", ros::init_options::NoSigintHandler );
    nh_ = new ros::NodeHandle();

    // Setup trajectory controller interface
    right_arm_client_ = MOVE3D_PTR_NAMESPACE::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> >(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(std::string("/r_arm_controller/joint_trajectory_action"), true));
    ROS_INFO("Waiting for right arm controllers to come up...");
    right_arm_client_->waitForServer();

    left_arm_client_ = MOVE3D_PTR_NAMESPACE::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> >(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(std::string("/l_arm_controller/joint_trajectory_action"), true));
    ROS_INFO("Waiting for left arm controllers to come up...");
    left_arm_client_->waitForServer();

    setActiveArm( right );

    // Subscribe to get current posture
    ros::Subscriber sub_r = nh_->subscribe<pr2_controllers_msgs::JointTrajectoryControllerState>( right_arm_topic_name_, 1,
                                                                                                  boost::bind( &Move3DRosHuman::GetJointState, this, _1,
                                                                                                               right_arm_joint_names_, right_arm_dof_ids_ ) );

    ros::Subscriber sub_l = nh_->subscribe<pr2_controllers_msgs::JointTrajectoryControllerState>( left_arm_topic_name_, 1,
                                                                                                  boost::bind( &Move3DRosHuman::GetJointState, this, _1,
                                                                                                               left_arm_joint_names_, left_arm_dof_ids_ ) );

    // Set module state
    setState( online );

    // Spin node
    ros::Rate spin_rate(joint_state_rate_);
    while (ros::ok())
    {
        // Process callbacks
        ros::spinOnce();
        // Spin
        spin_rate.sleep();
    }
}
