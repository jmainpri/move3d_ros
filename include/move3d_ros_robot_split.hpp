#ifndef MOVE3D_ROS_ROBOT_SPLIT_HPP
#define MOVE3D_ROS_ROBOT_SPLIT_HPP

#include <ros/ros.h>

#include "API/Device/robot.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"

class Move3DRosRobotSplit
{
public:

    Move3DRosRobotSplit();
    ~Move3DRosRobotSplit() { }

    bool split_in_trajectories() { return split_trajectories_; }
    void set_trajectories_folder(std::string folder)
    { trajectories_folder_ = folder; }
    void set_robot(Move3D::Robot* robot) { robot_ = robot; }

    bool loadSplits( std::string );
    bool saveToFiles();
    bool storeConfiguration( Move3D::confPtr_t q, double time );

    Move3D::Robot* getRobot() { return robot_; }

private:

    struct split_t {
        split_t(double t_0, double t_1 ){
            t_init_ = t_0;
            t_end_ = t_1;
        }
        double t_init_;
        double t_end_;
    };

    bool split_trajectories_;
    Move3D::Robot* robot_;
    Move3D::confPtr_t q_cur_;
    std::string trajectories_folder_;
    std::vector<split_t> splits_;
    motion_t configs_;
    std::vector<motion_t> trajectories_;
    ros::NodeHandle* nh_;
    int current_split_id_;
    double t_last_insert_;
};

#endif // MOVE3D_ROS_ROBOT_SPLIT_HPP
