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
#include "API/Graphic/drawModule.hpp"
#include "planner_handler.hpp"
#include "planner/plannerSequences.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "collision_space/collision_space_factory.hpp"
#include "utils/NumsAndStrings.hpp"

#include "qtMainInterface/mainwindow.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <time.h>
#include <sstream>

#include <ros/package.h>

using std::cout;
using std::endl;

Move3DRosReplanning::Move3DRosReplanning(QWidget *parent)
{
    // Set robot structure to 0
    robot_ = NULL;

    draw_joint_ = NULL;
    draw_rate_ = 10; // draws only the 10th time
    draw_execute_motion_ = false;

    send_to_robot_ = true;

    connect(this,  SIGNAL(drawAllWinActive()),global_w, SLOT(drawAllWinActive()), Qt::QueuedConnection);
    connect(this,  SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
}

Move3DRosReplanning::~Move3DRosReplanning()
{

}

bool Move3DRosReplanning::initReplanning(Move3D::confPtr_t q_goal, bool update)
{
    if( robot_ == NULL ){
        ROS_ERROR("No Move3D robot selected");
        return false;
    }

    if( !updateContext(update) ){
        ROS_ERROR("Could not update context");
        return false;
    }

    // GET STORED CONFIGURATIONS
    q_init_ = robot_->getCurrentPos();
    q_goal_ = q_goal->copy();

    // Set draw joint
    draw_joint_ = robot_->getJoint( ENV.getInt(Env::jntToDraw) );

    // INITIALIZE REPLANNING
    // current_human_traj_.resize( 0, 0 ); TODO add it for gathering data
    executed_trajectory_ = Move3D::Trajectory( robot_ );
    executed_trajectory_.setUseTimeParameter( true );
    executed_trajectory_.setUseConstantTime( false );
    executed_trajectory_.push_back( q_init_, 0.0 );

    // intiialize path
    path_ = Move3D::Trajectory( robot_ );

    // intialize time variables
    current_time_ = 0.0;
    global_discretization_ = 0.01; // time betweem configurations (choose a number that divides the simulation time step)
    time_along_current_path_ = 0.0;
    end_planning_ = false;
    current_motion_duration_ = 5.0; // General timelength between waypoints
    motion_duration_ = 5.0;

    time_step_ = 0.5; // Simulation step
    time_overhead_ = 0.25; // Initial estimate

    // Clear trajectory to draw
    global_linesToDraw.clear();

    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "Draw3DTrajs", boost::bind( &g3d_draw_3d_lines ) );
        global_DrawModule->enableDrawFunction( "Draw3DTrajs" );
    }

    return true;
}

void Move3DRosReplanning::setActiveDofs()
{
    std::vector<int> active_joints = traj_optim_get_planner_joints();

//    for(unsigned int i=0;i<active_joints.size();i++)
//    {
//        cout << "active joints [" << i << "]: " << active_joints[i] << endl;
//    }

    const std::vector<Move3D::Joint*>& joints = robot_->getJoints();

    cout << __PRETTY_FUNCTION__ << endl;

    active_dofs_.clear();
    for(unsigned int i=0; i<active_joints.size(); i++ )
    {
        active_dofs_.push_back( joints[active_joints[i]]->getIndexOfFirstDof() );
    }

//    for(unsigned int i=0;i<active_dofs_.size();i++)
//    {
//        cout << "active dofs [" << i << "]: " << active_dofs_[i] << endl;
//    }
}

bool Move3DRosReplanning::processTime() const
{
    cout << "MOTION DURATION : " << motion_duration_ << endl;
    cout << "CURRENT TIME : " << current_time_ << endl;
    cout << "TIME LEFT : " << current_motion_duration_ << endl;
    cout << "TIME ALONG CURRENT TRAJ : " << time_along_current_path_ << endl;

    if( motion_duration_ - current_time_ < 1e-2 )
        // inferior to a hundredth of a second
        return false;

    return true;
}

bool Move3DRosReplanning::updateContext(bool update_robot)
{
    try
    {
        context_ = get_context_(); // Function getting the context
    }
    catch(...)
    {
        return false;
    }

    for( size_t i=0; i<int(context_.size()); i++ )
    {
        Move3D::Robot* robot = context_[i]->getRobot();
        if( !update_robot && ( robot == robot_ ) )
            // Do not update robot when update_robot is false
                continue;

        cout << "UPDATE ROBOT : " << robot->getName() << endl;

        Move3D::confPtr_t q = context_[i]->copy();
        robot->setAndUpdate(*q);
    }

    if( !ENV.getBool(Env::drawDisabled) )
        g3d_draw_allwin_active();

    return true;
}

bool Move3DRosReplanning::runStandardStomp( int iter )
{
    cout << __PRETTY_FUNCTION__ << endl;

    int nb_way_points = 100;
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, nb_way_points );
//    traj_optim_set_discretize( true );
//    traj_optim_set_discretization( 0.015 );

    robot_->setAndUpdate( *q_init_ );

    robot_->setInitPos( *q_init_->copy() );
    robot_->setGoalPos( *q_goal_->copy() );

    if( iter == 0 )
    {
        traj_optim_clear_buffer();
        traj_optim_set_use_extern_trajectory( false );
        position_buffer_.clear();
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

        // Set buffer for smoothness computation
        if( time_along_current_path_ > 0.0 )
        {
            double dt = current_motion_duration_ / double(nb_way_points);
            int nb_config = 7; // diff_rule

            setActiveDofs();

            position_buffer_.clear();
            for(int i=0; i<nb_config; i++){
                Move3D::confPtr_t q = path_.configAtTime( time_along_current_path_-double(nb_config-i)*dt );
                position_buffer_.push_back( q->getEigenVector( active_dofs_ ) );
                // cout << "vect [" << i << "]: " << buffer[i].transpose() << endl;
            }

            traj_optim_set_buffer( position_buffer_ );

            if( !q_init_->equal( *path_.configAtTime( time_along_current_path_ ) ) ){
                ROS_ERROR("No Move3D human selected");
                return false;
            }
        }
    }

    double traj_duration = current_motion_duration_ > 0.0 ? current_motion_duration_ : 0.1;

    // SET PLANNING STOP CONDITIONS
    traj_optim_set_use_iteration_limit(false);
    PlanEnv->setBool(PlanParam::trajStompWithTimeLimit, true );
    PlanEnv->setDouble(PlanParam::trajStompTimeLimit, time_step_ - time_overhead_ );

    // PlanEnv->setBool(PlanParam::trajStompWithTimeLimit, false );
    // traj_optim_set_use_iteration_limit(true);
    // traj_optim_set_iteration_limit( PlanEnv->getInt(PlanParam::stompMaxIteration) );

    cout << "traj duration : " << traj_duration << endl;
    PlanEnv->setDouble( PlanParam::trajDuration, traj_duration  );

    traj_optim_resetInit();
    traj_optim_reset_collision_space();
    traj_optim_add_human_to_collision_space(true);

    traj_optim_runStomp(0);

    path_ = global_optimizer->getBestTraj();

    if( !path_.getUseTimeParameter() )
    {
        ROS_ERROR("path is not time parametrized");
    }
    return true;
}

void Move3DRosReplanning::execute( int iter, Move3D::Trajectory& path )
{
    cout << __PRETTY_FUNCTION__ << endl;
//    path.replaceP3dTraj();

    // Get alpha (slowing factor)
     double alpha = checkVelocityConstraints( path );

    // Set time correctly
    if( alpha > 1.)
    {
        double new_duration = current_motion_duration_ * alpha;
        motion_duration_ += new_duration - current_motion_duration_;
        current_motion_duration_ = new_duration;
        end_planning_ = true;
    }

    robot_trajectory_ = Move3D::Trajectory(robot_);
    robot_trajectory_.setUseTimeParameter( true );
    robot_trajectory_.setUseConstantTime( false );
    robot_trajectory_.push_back( q_init_->copy(), 0.0 );

    Move3D::confPtr_t q;

    int nb_configs = time_step_ / global_discretization_;
    // global_discretization_ must be a multiple of time step
    double t = 0;

//    cout << "path time length : " << path_.getDuration() << endl;
//    cout << "path number of waypoints : " << path_.getNbOfViaPoints() << endl;
//    cout << "nb_configs : " << nb_configs << endl;

    for( int i=0; i<nb_configs; i++ )
    {
        // Find configurations of active human along the trajectory
        t += global_discretization_;

        // Get config at time on trajectory
        q = path.configAtTime( t );

        //cout << "append configuration (t = " << t << ")" << endl;

        // Add the configuration to the trajectory
        executed_trajectory_.push_back( q, global_discretization_ );

        // Add to robot trajectory
        robot_trajectory_.push_back( q, global_discretization_ );

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

            // Get config at time on trajectory
            q = path.configAtTime( t );

            // Add to robot trajectory
            executed_trajectory_.push_back( q, global_discretization_ );

            // Add to robot trajectory
            robot_trajectory_.push_back( q, global_discretization_ );
        }
        end_planning_ = true;
    }

    // Add the end configuration
    if( end_planning_ )
    {
        cout << "END PLANNING" << endl;

        double dt = motion_duration_ - ( t + current_time_ );
        t += dt;

        // Add to executed traj
        executed_trajectory_.push_back( q_goal_->copy(), dt );

        // Add to robot trajectory
        robot_trajectory_.push_back( q_goal_->copy(), dt );
    }

    // Add trajectory to draw
    if( draw_joint_ != NULL && !ENV.getBool(Env::drawDisabled) )
    {
        global_linesToDraw.clear();
        global_linesToDraw.push_back( std::make_pair( Eigen::Vector3d(1, 0, 0), path.getJointPoseTrajectory( draw_joint_ ) ) );
    }
    else
        cout << "cannot draw trajectory" << endl;

    // Set time
    time_along_current_path_ = t;
    current_time_ += time_along_current_path_;
    current_motion_duration_ -= time_along_current_path_;

    // Set to previous q_init_
    robot_->setAndUpdate( *q_init_ );

    // Set last configuration as q_init_
    q_init_ = q->copy();

    cout << "End execute" << endl;
}

double Move3DRosReplanning::checkVelocityConstraints( Move3D::Trajectory& traj )
{
//      p3d_set_dof_velocity_max      1.5 # 0.522 # max  2.088
//      p3d_set_dof_velocity_max      1.52 # 0.522  # max 2.082
//      p3d_set_dof_velocity_max      1.5 # 0.8175   # max 3.27
//      p3d_set_dof_velocity_max      1.5 # 0.9  # max  3.3
//      p3d_set_dof_velocity_max      1.5 # 0.9 # max   3.3
//      p3d_set_dof_velocity_max      1.5 # 0.9 # max   3.078
//      p3d_set_dof_velocity_max      1.5 # 0.9 # max   3.6

    double secs_1 = ros::Time::now().toSec();

    std::vector<double> vel_limits(7, 0.6); // For pr2
//    vel_limits[0] = 0.5;
//    vel_limits[1] = 0.5;
//    vel_limits[2] = 0.7;
//    vel_limits[3] = 0.7;
//    vel_limits[4] = 0.7;
//    vel_limits[5] = 0.7;
//    vel_limits[6] = 0.7;

    smoothness_.setActiveDoFs( active_dofs_ );
    // smoothness_.setBuffer( position_buffer_ );
    smoothness_.clearBuffer();

    double alpha = 1.;
    bool vel_ok=true;

    Move3D::Trajectory executed_trajectory_constant_time(robot_);

    double dt_approx = 0.10; // 20Hz
    double duration = traj.getDuration();
    int nb_config   = duration / dt_approx;
    double dt       = duration / (nb_config-1);

    executed_trajectory_constant_time = Move3D::Trajectory(robot_);
    executed_trajectory_constant_time.setUseTimeParameter(true);
    executed_trajectory_constant_time.setUseConstantTime(true);
    executed_trajectory_constant_time.setDeltaTime( dt );

    for(int i=0; i<nb_config; i++)
    {
        Move3D::confPtr_t q(traj.configAtTime( double(i) * dt ));
        executed_trajectory_constant_time.push_back( q );
    }

    for(int i=0;i<100;i++)
    {
        vel_ok = true;

        std::vector<Eigen::VectorXd> control_costs;
        Eigen::MatrixXd traj_smooth = smoothness_.getSmoothedTrajectory( executed_trajectory_constant_time );
        smoothness_.getControlCosts( traj_smooth, control_costs, alpha * dt );

        for(size_t j=0; j<control_costs.size(); j++)
        {
            Eigen::VectorXd coeffs = Eigen::ArrayXd( control_costs[j] ).sqrt();

            // cout << "coeffs [" << j << " ] : " << coeffs.transpose() << endl;
            // cout << "control_costs [" << j << " ] : " << control_costs[j].transpose() << endl;

            if( coeffs.maxCoeff() > vel_limits[j] )
            {
                vel_ok = false;
                alpha += 2;
                executed_trajectory_constant_time.setDeltaTime( alpha * dt );
                break;
            }
        }

        if( vel_ok )
            break;
    }

    double secs_2 = ros::Time::now().toSec();
    double delta_time = secs_2 - secs_1;

    cout << "time : " << delta_time << " alpha = " << alpha << endl;
    // sleep(2);

//    std::ostringstream ss;
//    ss << std::setfill('0') << std::setw(3) << iter;

//    std::string folder = ros::package::getPath("move3d_ros") + "/data/vel_profiles/traj_" + ss.str() + "/";
//    smoothness_.saveAbsValuesToFile( executed_trajectory_constant_time, folder, executed_trajectory_constant_time.getDeltaTime() );

    // Set traj to execute
    traj = executed_trajectory_constant_time;

    return alpha;
}

void Move3DRosReplanning::saveExecutedTraj(int ith) const
{
    double dt_approx = 0.10; // 20Hz

    int nb_config = executed_trajectory_.getDuration() / dt_approx;
    double dt = executed_trajectory_.getDuration() / (nb_config-1);

    Move3D::Trajectory executed_trajectory_constant_time(robot_);
    executed_trajectory_constant_time.setUseTimeParameter(true);
    executed_trajectory_constant_time.setUseConstantTime(true);
    executed_trajectory_constant_time.setDeltaTime(dt);

    for(int i=0; i<nb_config; i++)
    {
        Move3D::confPtr_t q(executed_trajectory_.configAtTime( double(i) * dt ));
        executed_trajectory_constant_time.push_back( q );
    }

    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(4) << ith;

    std::string traj_name = ros::package::getPath("move3d_ros") +
            "/data/trajs/pr2/traj_" + ss.str() + ".traj";
    cout << "save executed_trajectory_" << traj_name << endl;
    executed_trajectory_constant_time.saveToFile( traj_name );
}

void Move3DRosReplanning::runReplanning()
{
    cout << __PRETTY_FUNCTION__ << endl;

    Move3D::SequencesPlanners pool( robot_ );
    std::vector<Move3D::confPtr_t> configs = pool.getStoredConfig();

    if( !configs.empty() )
    {
        std::vector<Move3D::confPtr_t> sequence;

//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );
//        sequence.push_back( configs[3] );
//        sequence.push_back( configs[4] );


        sequence.push_back( configs[3] );
        sequence.push_back( configs[0] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[1] );
        sequence.push_back( configs[3] );


        sequence.push_back( configs[0] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[1] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[0] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[2] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[0] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[1] );
        sequence.push_back( configs[3] );
        sequence.push_back( configs[0] );

        for(int j=0; j<sequence.size(); j++ )
        {
            initReplanning( sequence[j], send_to_robot_ ? true : j==0 );
            robot_->setAndUpdate( *q_init_ );

            ROS_INFO("STOMP NEW ITERATION");

            ros::Rate r( 1./time_step_ ); // 2 hz
            double delta_time = 0.0;

            for(int i=0; ros::ok() && (!PlanEnv->getBool(PlanParam::stopPlanner)); i++ )
            {
                double secs_1 = ros::Time::now().toSec();

                if( (i > 0) && send_to_robot_ )
                    send_trajectory_( robot_trajectory_, 0.0, end_planning_ );

                // if(i == 3)
                //    return;

                if( end_planning_ )
                {
                    ROS_INFO("Rend replanning: end");
                    break;
                }
                if( !processTime() ){ // advances simulation time
                    ROS_INFO("Rend replanning: time");
                    break;
                }

                cout << "____________________________________________________________" << endl;
                cout << "____________________________________________________________" << endl;
                cout << "____________________________________________________________" << endl;
                cout << "Replanning step " << i << " , TIME DELTA : " << delta_time << endl;

                if( !updateContext(false) ){ // get the human and possibily objects in the scene
                    ROS_ERROR("Replanning could not update");
                    return;
                }
                if( !runStandardStomp( i ) ) { // Rrun Stomp
                    ROS_ERROR("STOMP ERROR");
                    return;
                }

//                ros::Duration( time_step_ - time_overhead_ ).sleep();

                if( !PlanEnv->getBool(PlanParam::stopPlanner) )
                    execute( j, path_ );

                cout << "End of replanning step " << i << endl;

                double secs_2 = ros::Time::now().toSec();

                delta_time = secs_2 - secs_1;

                double time_remaning = time_step_ - delta_time;

                cout << " -- delta : " << delta_time << " sec." << endl;
                cout << " -- running  : " << time_step_ - time_overhead_  << " sec." << endl;
                cout << " -- remaning : " << time_remaning << " sec." << endl;
                cout << " -- overhead old : " << time_overhead_ << " sec." << endl;
                cout << " -- overhead new : " << time_overhead_ + ( 0.05 - time_remaning ) << " sec." << endl;

                time_overhead_ += ( 0.05 - time_remaning ); // automatic tunning of the overhead

                r.sleep();
            }

            sleep(1);

            saveExecutedTraj( j );

            robot_->setAndUpdate( *q_goal_ );
        }

        cout << "executed_trajectory_.size() : " << executed_trajectory_.size() << endl;

    }
    else
    {
        ROS_ERROR("No stored configs for replanning");
    }
}

void Move3DRosReplanning::run()
{
    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosReplanning::runReplanning, this ) );
    emit(selectedPlanner(QString("BoostFunction")));
}
