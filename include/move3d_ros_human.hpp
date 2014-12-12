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
#ifndef MOVE3D_ROS_HPP
#define MOVE3D_ROS_HPP

#include <QWidget>

#include <sensor_msgs/JointrState.h>
#include <ros/ros.h>

#include "API/Device/robot.hpp"

class Move3DRosHuman
{
    Q_OBJECT
    
public:

    explicit Move3DRosHuman(QWidget *parent = 0);
    ~Move3DRosHuman();

    void run();
    void initPr2();

    void executeElementaryMotion(Move3D::confPtr_t q_target);
    void executeMove3DTrajectory(const Move3D::Trajectory& traj);
    void executeLoadedMotionsThread();
    void loadMotions(std::string folder);
    void setActiveArm(arm_t arm);

signals:

    void selectedPlanner(QString);
    void drawAllWinActive();
    

private:

    Move3D::Robot* robot_;
    Move3D::confPtr_t q_cur_;
    bool update_robot_;
    std::vector<Move3D::Trajectory> move3d_trajs_;

    void GetJointState(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr arm_config,
                       std::vector<std::string> joint_names,
                       std::vector<int> dof_ids);

};

#endif // MOVE3D_ROS_HPP
