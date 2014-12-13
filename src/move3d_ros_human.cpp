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

#include "qtMainInterface/mainwindow.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <time.h>
#include <sstream>

using std::cout;
using std::endl;

Move3DRosHuman::Move3DRosHuman(QWidget *parent)
{
    // Set robot structure to 0
    robot_ = NULL;

//    joint_state_rate_ = 30;
    joint_state_received_ = 0;
    draw_rate_ = 10; // draws only the 10th time

    topic_name_ = "/mocap_human_joint_state";
    update_robot_ = false; // updates from sensor reading

    connect(this,  SIGNAL(drawAllWinActive()),global_w, SLOT(drawAllWinActive()), Qt::QueuedConnection);
}

Move3DRosHuman::~Move3DRosHuman()
{

}

bool Move3DRosHuman::initHuman()
{
    cout << __PRETTY_FUNCTION__ << endl;

    robot_ = Move3D::global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
    if( robot_ == NULL ){
        ROS_ERROR("No robot named pr2 in Move3D");
        return false;
    }

    joint_names_.clear();
    for(int i=0;i<robot_->getJoints().size();i++){
        joint_names_.push_back( robot_->getJoint(i)->getName() );
    }

    joint_map_.clear();
    for(int i=0;i<joint_names_.size();i++){
        Move3D::Joint* joint = robot_->getJoint(i);
        joint_map_[ joint->getName() ] = joint->getIndexOfFirstDof();
    }

    q_cur_ = robot_->getInitPos();
    return true;
}

static int joint_state_received=0;

void Move3DRosHuman::GetJointState( sensor_msgs::JointState::ConstPtr joint_config )
{
//    cout << __PRETTY_FUNCTION__ << endl;

    if( joint_names_.empty() ){
        ROS_ERROR("No human joint names");
        return;
    }

    // TODO fix this
    // Extract joint positions in the right order
//    if ( joint_config->name.size() != joint_names_.size() )
//    {
//        for (size_t idx=0; idx<joint_config->name.size(); idx ++)
//            cout << "joint_config->name[idx] : " << joint_config->name[idx] << endl;

//        for (size_t idx=0; idx<joint_names_.size(); idx ++)
//            cout << "joint_names_[idx] : " << joint_names_[idx] << endl;

//        ROS_ERROR("Malformed configuration update - skipping update");
//        return;
//    }

    if( robot_ == NULL ){
        ROS_ERROR("No Move3D human selected");
        return;
    }

    // Set the config
    Eigen::VectorXd new_arm_config( joint_names_.size() );
    std::vector<int> dof_ids( joint_names_.size() );

    try
    {
        for (size_t idx=0; idx<joint_config->name.size(); idx ++)
        {
            std::string name        = joint_config->name[idx];
            new_arm_config[idx]     = joint_config->position[idx];

            if( name == "PelvisTransX" )
                dof_ids[idx] = 6;
            else if( name == "PelvisTransY" )
                dof_ids[idx] = 7;
            else if( name == "PelvisTransZ" )
                dof_ids[idx] = 8;
            else if( name == "PelvisRotX" )
                dof_ids[idx] = 9;
            else if( name == "PelvisRotY" )
                dof_ids[idx] = 10;
            else if( name == "PelvisRotZ" )
                dof_ids[idx] = 11;
            else
                dof_ids[idx] = joint_map_[name];
        }
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

        joint_state_received += 1;

        // OPENGL DRAW
        if( (joint_state_received_) % draw_rate_ == 0 ) // 0 modulo k = 0
        {
            emit(drawAllWinActive());
            joint_state_received = 0;
        }
    }

    // Reset watchdog timer
    // arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);
}

ros::Subscriber Move3DRosHuman::subscribe_to_joint_angles(ros::NodeHandle* nh)
{
    cout << __PRETTY_FUNCTION__ << endl;

    if(!initHuman())
    {
        cout << "Error initializing human" << endl;
        return ros::Subscriber();
    }


    nh_ = nh;

    cout << "Subscribe to topic : " << topic_name_ << endl;

    // Subscribe to get current posture
    return nh_->subscribe<sensor_msgs::JointState>( topic_name_, 1, boost::bind( &Move3DRosHuman::GetJointState, this, _1) );
}
