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

#include <ros/ros.h>

#include "API/Device/robot.hpp"

#include "move3d_ros_robot.hpp"
#include "move3d_ros_human.hpp"
#include "move3d_ros_replanning.hpp"

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

    void initPr2();

    void runPr2Backend();
    void runHumanTracking();
    void runReplanning();

    void startNode();

    std::vector<Move3D::confPtr_t> getContext();
    bool sendTrajectory(const Move3D::Trajectory& trajectory, double time, bool wait );


public slots:

    void start();
    void setState(module_state_t state);
    void loadMotions();
    void executeLoadedMotions();
    bool gotoInit();


signals:

    void selectedPlanner(QString);
    void drawAllWinActive();
    

private:

    Ui::Move3DRosGui *ui_;

    int spin_rate_;

    int draw_rate_;
    bool draw_human_update_;
    bool draw_robot_update_;

    bool run_human_backend_;
    bool run_robot_backend_;
    bool run_replanning_;

    MOVE3D_PTR_NAMESPACE::shared_ptr<Move3DRosRobot> robot_backend_;
    MOVE3D_PTR_NAMESPACE::shared_ptr<Move3DRosHuman> human_joint_state_;
    MOVE3D_PTR_NAMESPACE::shared_ptr<Move3DRosReplanning> replanning_;

    ros::NodeHandle* nh_;
};

#endif // MOVE3D_ROS_HPP
