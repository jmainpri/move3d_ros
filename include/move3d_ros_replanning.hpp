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
#ifndef MOVE3D_ROS_REPLANNING_HPP
#define MOVE3D_ROS_REPLANNING_HPP

#include "qtLibrary.hpp"

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include "API/Device/robot.hpp"

class Move3DRosReplanning : public QObject
{
    Q_OBJECT
    
public:

    Move3DRosReplanning(QWidget *parent = 0);
    ~Move3DRosReplanning();

    bool setGetContextFunction(boost::function<std::vector<Move3D::confPtr_t>(void)> fct) { get_context_ = fct; }
    bool setSendTrajectoryFunction(boost::function<bool(const Move3D::Trajectory& trajectory, double time )> fct) { send_trajectory_ = fct; }

    void run();

    void setRobot(Move3D::Robot* robot) { robot_ = robot; }


signals:

    void selectedPlanner(QString);
    void drawAllWinActive();

private:

    Move3D::Robot* robot_;
    Move3D::Joint* draw_joint_;

    bool send_to_robot_;

    int draw_rate_;
    bool draw_execute_motion_;
    bool update_robot_;
    bool joint_state_received_;

    // Replanning variables
    bool end_planning_;
    double time_step_; // Simulation step
    double global_discretization_; // time betweem configurations (choose a number that divides the simulation time step)
    double current_time_;
    double time_along_current_path_;
    double motion_duration_; // TODO REMOVE
    double current_motion_duration_;
    // double current_discretization_; // TODO REMOVE

    double time_last_sent_;
    double time_overhead_;

    std::vector<Move3D::confPtr_t> context_;
    Move3D::confPtr_t q_init_;
    Move3D::confPtr_t q_goal_;
    Move3D::Trajectory executed_trajectory_;
    Move3D::Trajectory path_;
    std::vector<int> active_dofs_; // TODO set this in constructor

    boost::function<std::vector<Move3D::confPtr_t>(void)> get_context_;
    boost::function<bool(const Move3D::Trajectory& trajectory, double time )> send_trajectory_;

    void runReplanning();
    bool runStandardStomp( int iter );
    void execute( const Move3D::Trajectory& trajectory );
    bool updateContext( bool update_robot );
    bool initReplanning( Move3D::confPtr_t q_goal, bool update );
    void setActiveDofs();
    bool processTime() const;
};

#endif // MOVE3D_ROS_REPLANNING_HPP
