#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "teleop_robot/clik_planner.h"
#include <moveit/robot_model/robot_model.h>
#include <constrained_ik/basic_kin.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace teleop_robot
{

class RobotInterface
{
public:
  RobotInterface(const std::string& group);

  bool planAndMove(const std::vector<double>& seedv,
                   const Eigen::Affine3d& target_pose);

private:
  // Robot Descriptions and Solvers
  moveit::core::RobotModelPtr robot_model_;
  constrained_ik::basic_kin::BasicKin kin_;
  teleop_robot::ClikSolver solver_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  //
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  // Joint info
  std::vector<std::string> joint_names_;
};

}

#endif // ROBOT_INTERFACE_H

