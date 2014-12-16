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
#ifndef MOVE3D_ROS_HUMAN_HPP
#define MOVE3D_ROS_HUMAN_HPP

#include "qtLibrary.hpp"

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include "API/Device/robot.hpp"

#include <boost/thread/mutex.hpp>

class Move3DRosHuman : public QObject
{
    Q_OBJECT
    
public:

    Move3DRosHuman(QWidget *parent = 0);
    ~Move3DRosHuman();

    bool initHuman();
    void setUpdate(bool update) { update_robot_= update; }
    bool subscribe_to_joint_angles(ros::NodeHandle* nh);
    Move3D::confPtr_t get_current_conf();


signals:

    void drawAllWinActive();

private:

    Move3D::Robot* robot_;
    Move3D::confPtr_t q_cur_;
    std::vector<std::string> joint_names_;
    std::map<std::string,int> joint_map_;
    int draw_rate_;
    bool update_robot_;
    int joint_state_received_;
    std::string topic_name_;
    void GetJointState(sensor_msgs::JointState::ConstPtr arm_config );
    ros::Subscriber sub_;
    ros::NodeHandle* nh_;
    boost::mutex io_mutex_;

};

#endif // MOVE3D_ROS_HUMAN_HPP
