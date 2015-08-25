#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "teleop_robot/clik_planner.h"
#include <moveit/robot_model/robot_model.h>
#include <constrained_ik/basic_kin.h>
#include <ros/ros.h>

namespace teleop_robot
{

inline Eigen::VectorXd fromVector(const std::vector<double>& vec)
{
  return Eigen::VectorXd::Map(vec.data(), vec.size());
}

class RobotInterface
{
public:
  RobotInterface(const std::string& group);

  bool planAndMove(const std::vector<double>& seedv,
                   const Eigen::Affine3d& target_pose,
                   ros::Publisher& pub);

  bool fk(const Eigen::VectorXd& joints, Eigen::Affine3d& pose) const;

  bool ik(const Eigen::Affine3d& pose, const Eigen::VectorXd& seed,
          Eigen::VectorXd& solution) const;

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

