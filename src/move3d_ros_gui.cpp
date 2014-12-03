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
#include "planner/plannerSequences.hpp"

#include "qtMainInterface/mainwindow.hpp"
#include "qtOpenGL/glwidget.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
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

    connect(ui_->pushButtonStart ,SIGNAL(clicked()), this,SLOT(start()));
    connect(ui_->pushButtonLoadTrajs,  SIGNAL(clicked()), this, SLOT(loadMotions()));
    connect(ui_->pushButtonExecTrajs,  SIGNAL(clicked()), this, SLOT(executeLoadedMotions()));
    connect(this,  SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
    connect(this,  SIGNAL(drawAllWinActive()),global_w, SLOT(drawAllWinActive()), Qt::QueuedConnection);

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

Move3DRosGui::~Move3DRosGui()
{
    delete ui_;
}

void Move3DRosGui::initPr2()
{
    cout << __PRETTY_FUNCTION__ << endl;

    if( robot_ == NULL ){
        ROS_ERROR("No robot named pr2 in Move3D");
        return;
    }

    right_arm_joint_names_.resize(7);
    right_arm_joint_names_[0] = "r_shoulder_pan_joint";
    right_arm_joint_names_[1] = "r_shoulder_lift_joint";
    right_arm_joint_names_[2] = "r_upper_arm_roll_joint";
    right_arm_joint_names_[3] = "r_elbow_flex_joint";
    right_arm_joint_names_[4] = "r_forearm_roll_joint";
    right_arm_joint_names_[5] = "r_wrist_flex_joint";
    right_arm_joint_names_[6] = "r_wrist_roll_joint";

    left_arm_joint_names_.resize(7);
    left_arm_joint_names_[0] = "l_shoulder_pan_joint";
    left_arm_joint_names_[1] = "l_shoulder_lift_joint";
    left_arm_joint_names_[2] = "l_upper_arm_roll_joint";
    left_arm_joint_names_[3] = "l_elbow_flex_joint";
    left_arm_joint_names_[4] = "l_forearm_roll_joint";
    left_arm_joint_names_[5] = "l_wrist_flex_joint";
    left_arm_joint_names_[6] = "l_wrist_roll_joint";

    right_arm_dof_ids_.resize(7);
    right_arm_dof_ids_[0] = robot_->getJoint("right-Arm1")->getIndexOfFirstDof();
    right_arm_dof_ids_[1] = robot_->getJoint("right-Arm2")->getIndexOfFirstDof();
    right_arm_dof_ids_[2] = robot_->getJoint("right-Arm3")->getIndexOfFirstDof();
    right_arm_dof_ids_[3] = robot_->getJoint("right-Arm4")->getIndexOfFirstDof();
    right_arm_dof_ids_[4] = robot_->getJoint("right-Arm5")->getIndexOfFirstDof();
    right_arm_dof_ids_[5] = robot_->getJoint("right-Arm6")->getIndexOfFirstDof();
    right_arm_dof_ids_[6] = robot_->getJoint("right-Arm7")->getIndexOfFirstDof();

    left_arm_dof_ids_.resize(7);
    left_arm_dof_ids_[0] = robot_->getJoint("left-Arm1")->getIndexOfFirstDof();
    left_arm_dof_ids_[1] = robot_->getJoint("left-Arm2")->getIndexOfFirstDof();
    left_arm_dof_ids_[2] = robot_->getJoint("left-Arm3")->getIndexOfFirstDof();
    left_arm_dof_ids_[3] = robot_->getJoint("left-Arm4")->getIndexOfFirstDof();
    left_arm_dof_ids_[4] = robot_->getJoint("left-Arm5")->getIndexOfFirstDof();
    left_arm_dof_ids_[5] = robot_->getJoint("left-Arm6")->getIndexOfFirstDof();
    left_arm_dof_ids_[6] = robot_->getJoint("left-Arm7")->getIndexOfFirstDof();

    right_arm_topic_name_   = "/r_arm_controller/state";
    left_arm_topic_name_    = "/l_arm_controller/state";

    q_cur_ = robot_->getInitPos();
}

void Move3DRosGui::GetJointState(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr joint_config,
                                 std::vector<std::string> joint_names,
                                 std::vector<int> dof_ids)
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

void Move3DRosGui::setState(module_state_t state)
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

void Move3DRosGui::loadMotions()
{
    std::string folder = std::string(getenv("HOME_MOVE3D")) + "/trajs";

    // std::string folder = std::string(getenv("HOME_MOVE3D")) + "/../move3d-launch/launch_files";
    //    loadMotions( folder );

    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosGui::loadMotions, this, folder ) );
    emit(selectedPlanner(QString("BoostFunction")));
}

void Move3DRosGui::loadMotions(std::string folder)
{
    cout << __PRETTY_FUNCTION__ << endl;

    std::stringstream ss;
    std::vector<std::string> files;

    int nb_trajs = 12;
    for(int i=0; i<nb_trajs; i++ )
    {
        ss.str("");
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        files.push_back( folder + "/" + ss.str() );
        cout << "add file : " << files[i] << endl;
    }

    if( robot_ != NULL )
    {
        Move3D::SequencesPlanners pool( robot_ );
        pool.loadTrajsFromFile( files );
        pool.playTrajs();

        move3d_trajs_ = pool.getBestTrajs();
    }
}

void Move3DRosGui::executeLoadedMotions()
{
    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosGui::executeLoadedMotionsThread, this ) );
    emit(selectedPlanner(QString("BoostFunction")));
}

void Move3DRosGui::executeLoadedMotionsThread()
{
    cout << __PRETTY_FUNCTION__ << endl;

    if(move3d_trajs_.empty() )
    {
        ROS_ERROR( "No loaded trajectory" );
        return;
    }
    if( !move3d_trajs_[0].getUseTimeParameter() )
    {
        ROS_ERROR( "Move3D traj is not a time parametrized trajectory" );
        return;
    }

    executeElementaryMotion( move3d_trajs_[0].configAtTime(0.0) );

    for(size_t i=0; i<move3d_trajs_.size(); i++ )
    {
        Move3D::confPtr_t q_cur = q_cur_->copy();

//        Eigen::VectorXd q_1 = q_cur->getEigenVector( active_dof_ids_ );
//        Eigen::VectorXd q_2 = move3d_trajs_[i].configAtTime(0.0)->getEigenVector( active_dof_ids_ );

//        if( (q_1 - q_2).norm() > 0.4 )
//        {
//            ROS_ERROR( "Move3D traj is not starting at proper config, dist : %f", (q_1 - q_2).norm() );
//            return;
//        }

        executeMove3DTrajectory( move3d_trajs_[i] );
    }
}

void Move3DRosGui::executeMove3DTrajectory(const Move3D::Trajectory& traj)
{
    cout << __PRETTY_FUNCTION__ << endl;

    if( !traj.getUseTimeParameter() )
    {
        ROS_ERROR( "Move3D traj is not a time parametrized trajectory" );
        return;
    }

    pr2_controllers_msgs::JointTrajectoryGoal command;

    // Populate command
    command.trajectory.joint_names = active_joint_names_;
    command.trajectory.header.stamp = ros::Time::now();

    // Start trajectory
    double t = 0.0;
    double time_length = traj.getTimeLength();
    double dt = .2; // 100 ms (10 Hz)

    std::vector<double> config(active_dof_ids_.size());

    while( true )
    {
        Move3D::confPtr_t q = traj.configAtTime( t );

        // Get configuration
        for( int i=0; i<int(active_dof_ids_.size()); i++ )
            config[i] = (*q)[ active_dof_ids_[i] ];

        // Check configuration
        if( !command.trajectory.points.empty() )
            for( int i=0; i<int(config.size()); i++)
            {
                double diff = config[i] - command.trajectory.points.back().positions[i];
                double error = std::fabs( diff_angle( config[i] , command.trajectory.points.back().positions[i] ) - diff );
                if( error > 1e-6 ){
                    cout << "error at " << i << " by " << error << " in " << __PRETTY_FUNCTION__ << endl;
                }
            }

        // Populate points
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = config;
        point.velocities.clear(); // Clear for interpolation

        // Set the execution time
        point.time_from_start = ros::Duration( t );

        // Add point
        command.trajectory.points.push_back( point );

        t += dt; // increase time
        if( t > time_length ){ // TODO make sure to end at the end point
            break;
        }
    }

    // Command the arm
    active_arm_client_->sendGoal( command );

    // Wait until end of execution
    while(!active_arm_client_->getState().isDone() && ros::ok())
    {
        usleep(50000);
    }
}

void Move3DRosGui::executeElementaryMotion( Move3D::confPtr_t q_target )
{
    cout << __PRETTY_FUNCTION__ << endl;

    Move3D::confPtr_t q_cur = q_cur_->copy();


    // Get current configuration
    std::vector<double> config_current(active_dof_ids_.size());
    for( int j=0; j<int(active_dof_ids_.size()); j++ )
        config_current[j] = (*q_cur)[ active_dof_ids_[j] ];

    // Get target configuration
    std::vector<double> config_target(active_dof_ids_.size());
    for( int j=0; j<int(active_dof_ids_.size()); j++ )
        config_target[j] = (*q_target)[ active_dof_ids_[j] ];

    cout << "Filling command" << endl;

    pr2_controllers_msgs::JointTrajectoryGoal command;

    double execution_timestep = 10.0;

    // Populate command
    command.trajectory.joint_names = active_joint_names_;
    command.trajectory.header.stamp = ros::Time::now();

    // Populate target point
    trajectory_msgs::JointTrajectoryPoint start_point;
    start_point.positions = config_current;
    start_point.velocities.resize(start_point.positions.size(), 0.0);
    start_point.time_from_start = ros::Duration(0.0);

    // Populate target point
    trajectory_msgs::JointTrajectoryPoint target_point;
    target_point.positions = config_target;
    target_point.velocities.resize(target_point.positions.size(), 0.0);
    // Set the execution time
    target_point.time_from_start = ros::Duration(execution_timestep);

    cout << "sending command" << endl;

    // Add point
    command.trajectory.points.push_back(target_point);

    // Command the arm
    active_arm_client_->sendGoal(command);

    // Wait until end of execution
    while(!active_arm_client_->getState().isDone() && ros::ok())
    {
        usleep(50000);
    }
}

void Move3DRosGui::setActiveArm(arm_t arm)
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
                                                                                                boost::bind( &Move3DRosGui::GetJointState, this, _1,
                                                                                                             right_arm_joint_names_, right_arm_dof_ids_ ) );

    ros::Subscriber sub_l = nh_->subscribe<pr2_controllers_msgs::JointTrajectoryControllerState>( left_arm_topic_name_, 1,
                                                                                                boost::bind( &Move3DRosGui::GetJointState, this, _1,
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

void Move3DRosGui::start()
{
    boost::thread t( boost::bind( &Move3DRosGui::run, this ) );

//    while( true )
//    {
//        usleep(200000);

//        if(!ENV.getBool(Env::isRunning))
//            global_w->getOpenGL()->updateGL();
//    }

//    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosGui::run, this ) );
//    emit(selectedPlanner(QString("BoostFunction")));
}
