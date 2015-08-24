#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "teleop_robot/clik_planner.h"
#include <moveit/robot_model/robot_model.h>
#include <constrained_ik/basic_kin.h>
#include <ros/ros.h>

namespace teleop_robot
{

class RobotInterface
{
public:
  RobotInterface(const std::string& group);

  bool planAndMove(const std::vector<double>& seedv,
                   const Eigen::Affine3d& target_pose,
                   ros::Publisher& pub);

private:
  // Robot Descriptions and Solvers
  moveit::core::RobotModelPtr robot_model_;
  constrained_ik::basic_kin::BasicKin kin_;
  teleop_robot::ClikSolver solver_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  // Joint info
  std::vector<std::string> joint_names_;
};

}

#endif // ROBOT_INTERFACE_H

