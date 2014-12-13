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

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include "API/Device/robot.hpp"

namespace Ui {
class Move3DRosGui;
}

class Move3DRosGui : public QWidget
{
    Q_OBJECT
    
public:
    enum arm_t {left, right} arm_;
    enum module_state_t {online, offline};

    explicit Move3DRosGui(QWidget *parent = 0);
    ~Move3DRosGui();

    void run();
    void initPr2();

    void startHumanTracking();

    void executeElementaryMotion(Move3D::confPtr_t q_target);
    void executeMove3DTrajectory(const Move3D::Trajectory& traj);
    void executeLoadedMotionsThread();
    void loadMotions(std::string folder);
    void setActiveArm(arm_t arm);


public slots:

    void start();
    void setState(module_state_t state);
    void loadMotions();
    void executeLoadedMotions();


signals:

    void selectedPlanner(QString);
    void drawAllWinActive();
    

private:

    Ui::Move3DRosGui *ui_;

    int joint_state_rate_;
    int joint_state_received_;
    int draw_rate_;

    std::string active_state_topic_name_;
    std::string right_arm_topic_name_;
    std::string left_arm_topic_name_;

    std::vector<std::string> right_arm_joint_names_;
    std::vector<std::string> left_arm_joint_names_;
    std::vector<std::string> active_joint_names_;

    std::vector<int> right_arm_dof_ids_;
    std::vector<int> left_arm_dof_ids_;
    std::vector<int> active_dof_ids_;


    Move3D::Robot* robot_;
    Move3D::confPtr_t q_cur_;
    bool update_robot_;
    std::vector<Move3D::Trajectory> move3d_trajs_;

    void GetJointState(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr arm_config,
                       std::vector<std::string> joint_names,
                       std::vector<int> dof_ids);

    ros::NodeHandle* nh_;
    MOVE3D_PTR_NAMESPACE::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> > right_arm_client_;
    MOVE3D_PTR_NAMESPACE::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> > left_arm_client_;
    MOVE3D_PTR_NAMESPACE::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> > active_arm_client_;
};

#endif // MOVE3D_ROS_HPP
