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

#include "move3d_ros_human.hpp"

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

    connect(ui_->pushButtonStart ,     SIGNAL(clicked()), this,SLOT(start()));
    connect(ui_->pushButtonLoadTrajs,  SIGNAL(clicked()), this, SLOT(loadMotions()));
    connect(ui_->pushButtonExecTrajs,  SIGNAL(clicked()), this, SLOT(executeLoadedMotions()));

    connect(this,  SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
    connect(this,  SIGNAL(drawAllWinActive()),global_w, SLOT(drawAllWinActive()), Qt::QueuedConnection);

//    // Set robot structure to 0
//    move3d_trajs_.clear();

//    // Set all active joint ids and names to 0
//    active_joint_names_.clear();
//    active_dof_ids_.clear();

    // GUI SPIN RATE
    spin_rate_ = 30;

    draw_rate_ = 10; // draws only the 10th time
    draw_human_update_ = false;
    draw_robot_update_ = false; // updates from sensor reading
}

Move3DRosGui::~Move3DRosGui()
{
    delete ui_;
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
    robot_backend_->loadMotions();
}

void Move3DRosGui::executeLoadedMotions()
{
    robot_backend_->executeLoadedMotions();
}

void Move3DRosGui::runReplanning()
{
     cout << __PRETTY_FUNCTION__ << endl;
}

void Move3DRosGui::runHumanTracking()
{
     cout << __PRETTY_FUNCTION__ << endl;

    // Update human joint angles
    human_joint_state_ = MOVE3D_PTR_NAMESPACE::shared_ptr<Move3DRosHuman>(new Move3DRosHuman());
    human_joint_state_->subscribe_to_joint_angles(nh_);
    human_joint_state_->setUpdate( draw_human_update_ );
}

void Move3DRosGui::runPr2Backend()
{
    cout << __PRETTY_FUNCTION__ << endl;

    // Update robot joint angles
    robot_backend_ = MOVE3D_PTR_NAMESPACE::shared_ptr<Move3DRosRobot>(new Move3DRosRobot());
    robot_backend_->run_pr2_backend(nh_);
    robot_backend_->setUpdate( draw_robot_update_ );
}

void Move3DRosGui::startNode()
{
    int argc = 0;
    char **argv = NULL;
    ros::init( argc, argv, "move3d_pr2", ros::init_options::NoSigintHandler );
    nh_ = new ros::NodeHandle();

    ros::NodeHandle nhp("~");
    nhp.param(std::string("run_human_tracking"), run_human_backend_,    bool(false));
    nhp.param(std::string("run_pr2_backend"),    run_pr2_backend_,      bool(false));
    nhp.param(std::string("run_replanning"),     run_replanning_,       bool(false));
    nhp.param(std::string("draw_human_update"),  draw_human_update_,    bool(false));
    nhp.param(std::string("draw_robot_update"),  draw_robot_update_,    bool(false));
    // nhp.param(std::string("arm_config_topic"), arm_config_topic, std::string("/l_arm_controller/state"));
    // nhp.param(std::string("arm_command_action"), arm_command_action, std::string("/l_arm_controller/joint_trajectory_action"));

    ui_->checkBoxRunHumanBackEnd->setChecked(   run_human_backend_ );
    ui_->checkBoxRunPr2Backend->setChecked(     run_pr2_backend_ );
    ui_->checkBoxRunReplanning->setChecked(     run_replanning_ );

    if( run_pr2_backend_ )
        runPr2Backend();

    if( run_human_backend_ )
        runHumanTracking();

    if( run_replanning_ )
        runReplanning();

    // Set module state
    setState( online );

    cout << "start spinning node GUI" << endl;
    // Spin node
    ros::Rate spin_rate(spin_rate_);
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
    cout << __PRETTY_FUNCTION__ << endl;

    boost::thread t( boost::bind( &Move3DRosGui::startNode, this ) );

//    while( true )
//    {
//        usleep(200000);

//        if(!ENV.getBool(Env::isRunning))
//            global_w->getOpenGL()->updateGL();
//    }

//    global_plannerHandler->setExternalFunction( boost::bind( &Move3DRosGui::run, this ) );
//    emit(selectedPlanner(QString("BoostFunction")));
}
