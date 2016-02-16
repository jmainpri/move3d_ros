#include "move3d_ros_robot_split.hpp"
#include <libmove3d/planners/utils/misc_functions.hpp>
#include <libmove3d/planners/utils/NumsAndStrings.hpp>

using std::cout;
using std::endl;

Move3DRosRobotSplit::Move3DRosRobotSplit()
    : split_trajectories_(false),
      robot_(NULL),
      human_(NULL),
      done_splitting_robot_(true),
      done_splitting_human_(true) {}

bool Move3DRosRobotSplit::loadSplits(std::string filename) {
  std::vector<std::string> strings = load_strings_from_file(filename);
  if (strings.empty()) return false;

  if (strings.size() % 4 != 0) return false;

  splits_.clear();

  cout << "strings size : " << strings.size() << endl;

  for (size_t i = 0; i < strings.size(); i += 4) {
    cout << " i : " << i << endl;

    double t_sec, t_nano;
    convert_text_to_num<double>(t_sec, strings[i + 0], std::dec);
    convert_text_to_num<double>(t_nano, strings[i + 1], std::dec);
    ros::Time t_ini(t_sec, t_nano);

    convert_text_to_num<double>(t_sec, strings[i + 2], std::dec);
    convert_text_to_num<double>(t_nano, strings[i + 3], std::dec);
    ros::Time t_end(t_sec, t_nano);

    splits_.push_back(split_t(t_ini.toSec(), t_end.toSec()));

    cout << "split init : " << t_ini.toSec() << endl;
  }

  for (size_t i = 0; i < splits_.size(); i++) {
    cout << "splits " << i << " = t_ini: " << splits_[i].t_init_
         << " = t_end: " << splits_[i].t_end_ << endl;
  }

  split_trajectories_ = true;

  // Robot data
  robot_trajectories_.clear();
  robot_configs_.clear();
  robot_current_split_id_ = 0;

  // Human data
  human_trajectories_.clear();
  human_configs_.clear();
  human_current_split_id_ = 0;

  return true;
}

void Move3DRosRobotSplit::saveTrajectoriesToFile() {
  saveToFiles(robot_trajectories_, true);
  saveToFiles(human_trajectories_, false);
}

bool Move3DRosRobotSplit::saveToFiles(std::vector<motion_t>& trajectories,
                                      bool is_robot) {
  cout << "save to file, is robot :" << is_robot << endl;
  if (trajectories.empty()) {
    return false;
  }

  // Becarful the trajectory storage performs foward kinematics
  Move3D::Robot* robot = trajectories[0][0].second->getRobot();

  for (size_t i = 0; i < trajectories.size(); i++) {
    std::stringstream ss;
    ss.str("");
    if (is_robot) {
      ss << trajectories_folder_ << "/robot/robot_" << std::setw(3)
         << std::setfill('0') << i << ".traj";
    } else {
      ss << trajectories_folder_ << "/human/human_" << std::setw(3)
         << std::setfill('0') << i << ".traj";
    }
    double motion_duration = HRICS::motion_duration(trajectories[i]);
    cout << "motion_duration : " << motion_duration << endl;

    // Resample at 100 Hz constant dt
    Move3D::Trajectory traj(HRICS::motion_to_traj(trajectories[i], robot));

    // Save to file
    traj.saveToFile(ss.str());

    ROS_INFO(
        "SAVE TRAJECTORY %d for robot %s with %d waypoints, duration %f\n at "
        "%s",
        int(i),
        robot->getName().c_str(),
        traj.getNbOfViaPoints(),
        traj.getDuration(),
        ss.str().c_str());
  }

  trajectories.clear();
  return true;
}

bool Move3DRosRobotSplit::storeConfiguration(
    Move3D::confPtr_t q,
    int& current_split_id,
    double& t_last_insert,
    motion_t& configs,
    std::vector<motion_t>& trajectories) const {
  // Check if after the end of split
  double time = ros::Time::now().toSec();
  if (time > splits_[current_split_id].t_end_) {
    trajectories.push_back(configs);
    configs.clear();
    current_split_id++;
  }

  if (current_split_id >= splits_.size()) return false;

  // Check if after begining of split
  if (time >= splits_[current_split_id].t_init_) {
    double dt = 0.0;

    if (!configs.empty()) {
      dt = time - t_last_insert;
    }
    t_last_insert = time;
    //cout << "dt : " << dt << endl;
    configs.push_back(std::make_pair(dt, q->copy()));
  }
}

bool Move3DRosRobotSplit::storeRobotConfiguration(Move3D::confPtr_t q) {
  if (robot_current_split_id_ >= splits_.size()) {
    done_splitting_robot_ = true;
    return false;
  }

  return storeConfiguration(q,
                            robot_current_split_id_,
                            robot_t_last_insert_,
                            robot_configs_,
                            robot_trajectories_);
}

bool Move3DRosRobotSplit::storeHumanConfiguration(Move3D::confPtr_t q) {
  if (human_current_split_id_ >= splits_.size()) {
    done_splitting_human_ = true;
    return false;
  }

  return storeConfiguration(q,
                            human_current_split_id_,
                            human_t_last_insert_,
                            human_configs_,
                            human_trajectories_);
}
