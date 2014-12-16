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

#include "move3d_ros_replanning.hpp"


#include <std_msgs/String.h>

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/Graphic/drawCost.hpp"
#include "planner_handler.hpp"
#include "planner/plannerSequences.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "collision_space/collision_space_factory.hpp"

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

Move3DRosReplanning::Move3DRosReplanning(QWidget *parent)
{
    // Set robot structure to 0
    robot_ = NULL;

    draw_rate_ = 10; // draws only the 10th time
    draw_execute_motion_ = false;
    draw_joint_ = robot_->getJoint( 45 );

    connect(this,  SIGNAL(drawAllWinActive()),global_w, SLOT(drawAllWinActive()), Qt::QueuedConnection);
    connect(this,  SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
}

Move3DRosReplanning::~Move3DRosReplanning()
{

}

bool Move3DRosReplanning::initReplanning(Move3D::Robot* robot, Move3D::confPtr_t q_goal)
{
    robot_ = robot;

    if( robot_ == NULL ){
        ROS_ERROR("No Move3D robot selected");
        return false;
    }

    if( !updateContext() ){
        ROS_ERROR("Could not update context");
        return false;
    }

    // GET STORED CONFIGURATIONS
    q_goal_ = q_goal->copy();
    q_init_ = robot->getCurrentPos();

    // INITIALIZE REPLANNING
    // current_human_traj_.resize( 0, 0 ); TODO add it for gathering data
    executed_trajectory_ = Move3D::Trajectory(robot_);
    executed_trajectory_.setUseTimeParameter( true );
    executed_trajectory_.setUseConstantTime( false );
    executed_trajectory_.push_back(  q_init_, 0.0 );
    current_time_ = 0.0;
    time_step_ = 0.1; // Simulation step
    global_discretization_ = 0.01; // time betweem configurations (choose a number that divides the simulation time step)
    time_along_current_path_ = 0.0;
    end_planning_ = false;

    return true;
}

bool Move3DRosReplanning::updateContext()
{
    cout << "MOTION DURATION : " << motion_duration_ << endl;
    cout << "CURRENT TIME : " << current_time_ << endl;
    cout << "TIME LEFT : " << current_motion_duration_ << endl;
    cout << "TIME ALONG CURRENT TRAJ : " << time_along_current_path_ << endl;

    context_ = get_context_(); // Function getting the context

    for( size_t i=0; i<int(context_.size()); i++ )
    {
        Move3D::Robot* robot = context_[i]->getRobot();
        Move3D::confPtr_t q = context_[i]->copy();
        robot->setAndUpdate(*q);
    }
    return true;
}

bool Move3DRosReplanning::runStandardStomp( int iter )
{
    int nb_way_points = 100;
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, nb_way_points );
//    traj_optim_set_discretize( true );
//    traj_optim_set_discretization( 0.015 );

    robot_->setAndUpdate( *q_init_ );

    robot_->setInitPos( *q_init_->copy() );
    robot_->setGoalPos( *q_goal_->copy() );

    if( iter == 0 )
    {
        traj_optim_set_use_extern_trajectory( false );
    }
    else
    {
        // SET CURRENT TRAJECTORY AS INITIAL GUESS
        Move3D::Trajectory optimi_traj( robot_ );
        double dt = current_motion_duration_ / double(nb_way_points-1);
        for(int i=0; i<nb_way_points-1; i++) {
            optimi_traj.push_back( path_.configAtTime( time_along_current_path_ + double(i) * dt ) );
        }
        optimi_traj.push_back( path_.getEnd() );

        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( optimi_traj );
    }

    double traj_duration = current_motion_duration_ > 0.0 ? current_motion_duration_ : 0.1;

    // SET PLANNING STOP CONDITIONS
    PlanEnv->setBool(PlanParam::trajStompWithTimeLimit, true );
    PlanEnv->setDouble(PlanParam::trajStompTimeLimit, traj_duration - 0.02 );

    traj_optim_set_use_iteration_limit(false);
    // traj_optim_set_use_iteration_limit(true);
    // traj_optim_set_iteration_limit( PlanEnv->getInt(PlanParam::stompMaxIteration) );

    PlanEnv->setDouble( PlanParam::trajDuration, traj_duration  );

    // Set buffer for smoothness computation
    if( path_.size() > 0 && time_along_current_path_ > 0.0 )
    {
        double dt = current_motion_duration_ / double(nb_way_points);
        std::vector<Eigen::VectorXd> buffer;
        int nb_config = 7;
        for(int i=0; i<nb_config; i++){
            Move3D::confPtr_t q = path_.configAtTime( time_along_current_path_-double(nb_config-i)*dt );
            buffer.push_back( q->getEigenVector( active_dofs_ ) );
        }
        traj_optim_set_buffer( buffer );

        if( !q_init_->equal( *path_.configAtTime( time_along_current_path_ ) ) ){
            ROS_ERROR("No Move3D human selected");
            return false;
        }
    }

    traj_optim_resetInit();
    traj_optim_reset_collision_space();
    traj_optim_add_human_to_collision_space(true);

    traj_optim_runStomp(0);

    path_ = global_optimizer->getBestTraj();
    cout << "path.getUseTimeParameter() : " << path_.getUseTimeParameter() << endl;
    return true;
}

/**
void Move3DRosReplanning::execute(const Move3D::Trajectory& path, bool to_end)
{
//    path.replaceP3dTraj();

    if( path.getUseTimeParameter() )
        current_discretization_ = path.getDeltaTime();
    else
        current_discretization_ = current_motion_duration_ / double( path.getNbOfPaths() );

    Move3D::confPtr_t q;

    double time_factor = 10; // Slow down execution by factor

    int nb_configs = time_step_ / global_discretization_; // global_discretization_ must be a multiple of time step
    double t = 0;

    for( int i=0; i<nb_configs; i++ )
    {
        // Find configurations of active human along the trajectory
        t += global_discretization_;
        q = path.configAtTime( t );
        robot_->setAndUpdate( *q );

        // Add the configuration to the trajectory
        executed_trajectory_.push_back( q, global_discretization_ );

        // If the time exceeds the trajectory length
        // inferior to a hundredth of a second
        if( ( motion_duration_ - ( t + current_time_ ) ) < global_discretization_  )
        {
            end_planning_ = true;
            break;
        }
        if( draw_execute_motion_ )
        {
            drawAllWinActive();
            usleep( floor( global_discretization_ * 1e6 * time_factor ) );
        }
    }

    // If duration left is inferior to half a time step do not replan
    if( ( motion_duration_ - ( t + current_time_ ) ) <= (time_step_/2.) )
    {
        while( motion_duration_ - ( t + current_time_ ) > global_discretization_ )
        {
            t += global_discretization_;
            q = path.configAtTime( t );
            executed_trajectory_.push_back( q, global_discretization_ );
        }
        end_planning_ = true;
    }

    // Add the end configuration
    if( end_planning_ ) {
        double dt = motion_duration_ - ( t + current_time_ );
        t += dt;
        executed_trajectory_.push_back( q_goal_->copy(), dt );
    }

    time_along_current_path_ = t;
    current_time_ += time_along_current_path_;
    current_motion_duration_ -= time_along_current_path_;

    q_init_ = q;

    cout << "End execute" << endl;
}
*/

void Move3DRosReplanning::execute(const Move3D::Trajectory& path )
{
//    path.replaceP3dTraj();

    Move3D::confPtr_t q;

    int nb_configs = time_step_ / global_discretization_; // global_discretization_ must be a multiple of time step
    double t = 0;

    for( int i=0; i<nb_configs; i++ )
    {
        // Find configurations of active human along the trajectory
        t += global_discretization_;

        // Add the configuration to the trajectory
        executed_trajectory_.push_back( path.configAtTime( t ), global_discretization_ );

        // If the time exceeds the trajectory length
        // inferior to a hundredth of a second
        if( ( motion_duration_ - ( t + current_time_ ) ) < global_discretization_  )
        {
            end_planning_ = true;
            break;
        }
    }

    // If duration left is inferior to half a time step do not replan
    if( ( motion_duration_ - ( t + current_time_ ) ) <= (time_step_/2.) )
    {
        while( motion_duration_ - ( t + current_time_ ) > global_discretization_ )
        {
            t += global_discretization_;
            executed_trajectory_.push_back( path.configAtTime( t ), global_discretization_ );
        }
        end_planning_ = true;
    }

    // Add the end configuration
    if( end_planning_ ) {
        double dt = motion_duration_ - ( t + current_time_ );
        t += dt;
        executed_trajectory_.push_back( q_goal_->copy(), dt );
    }

    time_along_current_path_ = t;
    current_time_ += time_along_current_path_;
    current_motion_duration_ -= time_along_current_path_;

    global_linesToDraw.push_back( std::make_pair( Eigen::Vector3d(1, 0, 0), path.getJointPoseTrajectory( draw_joint_ ) ) );

    q_init_ = q;

    cout << "End execute" << endl;
}

double Move3DRosReplanning::run(Move3D::confPtr_t q_goal)
{
    robot_->setAndUpdate( *q_init_ );

    for(int i=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updateContext(); i++ )
    {
        runStandardStomp( i );

        if( !PlanEnv->getBool(PlanParam::stopPlanner) )
            execute( path_ );

        drawAllWinActive();

//        cout << "wait for key" << endl;
//        cin.ignore();
//        path_.replaceP3dTraj();

    }

    drawAllWinActive();

    cout << "executed_trajectory_.size() : " << executed_trajectory_.size() << endl;
    return 0.0;
}
