#ifndef MOVE3D_ROS_HPP
#define MOVE3D_ROS_HPP

#include <QWidget>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <ros/ros.h>

namespace Ui {
class Move3DRosGui;
}

class Move3DRosGui : public QWidget
{
    Q_OBJECT
    
public:
    explicit Move3DRosGui(QWidget *parent = 0);
    ~Move3DRosGui();

    int init(int argc, char **argv);

public slots:
    void start();
    
private:
    std::vector<std::string> joint_names_;

    void GetPr2RArmState(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr arm_config);
    Ui::Move3DRosGui *ui_;
    ros::NodeHandle nh_;
};

#endif // MOVE3D_ROS_HPP
