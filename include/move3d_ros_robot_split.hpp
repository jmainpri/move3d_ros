#ifndef MOVE3D_ROS_ROBOT_SPLIT_HPP
#define MOVE3D_ROS_ROBOT_SPLIT_HPP

#include <ros/ros.h>

#include "API/Device/robot.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"

class Move3DRosRobotSplit {
 public:
  Move3DRosRobotSplit();
  ~Move3DRosRobotSplit() {}

  //! get the number of split trajectories
  bool split_in_trajectories() { return split_trajectories_; }

  //! set the trajectories forlder
  void set_trajectories_folder(std::string folder) {
    trajectories_folder_ = folder;
  }

  //! Load splits from the filename
  bool loadSplits(std::string filename);

  //! store the configurations at time t
  bool storeRobotConfiguration(Move3D::confPtr_t q);

  //! store the configurations at time t
  bool storeHumanConfiguration(Move3D::confPtr_t q);

  //! set the robot pointer
  void set_robot(Move3D::Robot* robot) {
    robot_ = robot;
    done_splitting_robot_ = false;
  }

  //! set the robot pointer
  void set_human(Move3D::Robot* human) {
    human_ = human;
    done_splitting_human_ = false;
  }

  //! save all trajectories to file
  void saveTrajectoriesToFile();

  //! get the robot pointer
  Move3D::Robot* robot() { return robot_; }

  //! get the robot pointer
  Move3D::Robot* human() { return human_; }

  //! true if done splitting
  bool done_splitting() const {
    return done_splitting_robot_ && done_splitting_human_;
  }

 private:
  struct split_t {
    split_t(double t_0, double t_1) {
      t_init_ = t_0;
      t_end_ = t_1;
    }
    double t_init_;
    double t_end_;
  };

  //! Save the robot trajectories to file
  bool saveToFiles(std::vector<motion_t>& trajectories, bool is_robot);

  //! Store a configuration
  bool storeConfiguration(Move3D::confPtr_t q,
                          int& current_split_id,
                          double& t_last_insert,
                          motion_t& configs,
                          std::vector<motion_t>& trajectories) const;

  bool split_trajectories_;
  Move3D::confPtr_t q_cur_;
  std::string trajectories_folder_;
  std::vector<split_t> splits_;

  ros::NodeHandle* nh_;

  // Robot variables
  Move3D::Robot* robot_;
  int robot_current_split_id_;
  double robot_t_last_insert_;
  motion_t robot_configs_;
  std::vector<motion_t> robot_trajectories_;
  bool done_splitting_robot_;

  // Human variables
  Move3D::Robot* human_;
  int human_current_split_id_;
  double human_t_last_insert_;
  motion_t human_configs_;
  std::vector<motion_t> human_trajectories_;
  bool done_splitting_human_;
};

#endif  // MOVE3D_ROS_ROBOT_SPLIT_HPP
