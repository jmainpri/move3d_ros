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

#include <cmath>
#include <time.h>

#include <sstream>


Move3DRosGui::Move3DRosGui(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::Move3DRosGui)
{
    ui_->setupUi(this);

    connect(ui_->pushButtonStart,SIGNAL(clicked()),this,SLOT(start()));
}

Move3DRosGui::~Move3DRosGui()
{
    delete ui_;
}

#define PR2_ARM_JOINTS 7

void Move3DRosGui::GetPr2RArmState(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr arm_config)
{
    Move3D::Robot* pr2 = Move3D::global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");

    // Extract joint positions in the right order
    if (arm_config->joint_names.size() != arm_config->actual.positions.size() || arm_config->joint_names.size() != PR2_ARM_JOINTS)
    {
        ROS_ERROR("Malformed configuration update - skipping update");
        return;
    }
    std::map<std::string, double> arm_configuration;
    for (size_t idx = 0; idx < arm_config->joint_names.size(); idx ++)
    {
        arm_configuration[arm_config->joint_names[idx]] = arm_config->actual.positions[idx];
    }
    //std::cout << "Got updated config: " << PrettyPrint(arm_configuration, true) << std::endl;
    // Set the config
    std::vector<double> new_arm_config(PR2_ARM_JOINTS);
    new_arm_config[0] = arm_configuration[joint_names_[0]];
    new_arm_config[1] = arm_configuration[joint_names_[1]];
    new_arm_config[2] = arm_configuration[joint_names_[2]];
    new_arm_config[3] = arm_configuration[joint_names_[3]];
    new_arm_config[4] = arm_configuration[joint_names_[4]];
    new_arm_config[5] = arm_configuration[joint_names_[5]];
    new_arm_config[6] = arm_configuration[joint_names_[6]];
//    current_arm_config_ = new_arm_config;


    // Reset watchdog timer
//    arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);


}

int Move3DRosGui::init(int argc, char **argv)
{
    ros::init(argc, argv, "move3d_pr2");

    ros::Subscriber sub = nh_.subscribe("/r_arm_controller/state", 1, &Move3DRosGui::GetPr2RArmState, this);
    ros::spin();

    return 0;
}

void Move3DRosGui::start()
{
    init(0, NULL);
}
