#include "teleop_robot/robot_interface.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <chrono>

#include "trajectory_msgs/JointTrajectory.h"

teleop_robot::RobotInterface::RobotInterface(const std::string& group)
  : ac_("joint_trajectory_action", true)
{
  robot_model_loader::RobotModelLoader loader;
  robot_model_ = loader.getModel();

  if (!robot_model_)
  {
    ROS_FATAL("Could not load robot model from parameter server");
    throw std::runtime_error("Could not load robot model");
  }

  if (!kin_.init(robot_model_->getJointModelGroup(group)))
  {
    ROS_FATAL("Could not initialize kinematics object");
    throw std::runtime_error("Could not intiailize kinematics object");
  }

  kin_.getJointNames(joint_names_);


  solver_.init(kin_);

}

bool teleop_robot::RobotInterface::planAndMove(const std::vector<double>& seedv,
                                               const Eigen::Affine3d& target_pose)
{
   auto t2 = std::chrono::steady_clock::now();

   // attempt to calculate IK
   Eigen::VectorXd seed(seedv.size());
   Eigen::VectorXd result(seedv.size());
   for (unsigned i = 0; i < seedv.size(); ++i)
     seed(i) = seedv[i];

   try
   {
     solver_.calcInvKin(target_pose, seed, planning_scene_, result);
   }
   catch (std::exception& e)
   {
     ROS_ERROR_STREAM(e.what());
     ROS_ERROR_STREAM("POSE\n" << target_pose.matrix());
     return false;
   }
    auto t3 = std::chrono::steady_clock::now();

  // if success, construct action goal with trajectory of size 1
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory traj;
  traj.header.frame_id = "world";
  traj.header.stamp = ros::Time::now();
  traj.joint_names = joint_names_;

  // Calculate dt
  const static double vel[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0};
  double max_t = 0.0;

  for (unsigned i = 0; i < seedv.size(); ++i)
  {
    double dtheta = std::abs(seed(i) - result(i)) / vel[i];
    max_t = std::max(dtheta, max_t);
  }

  // to pt
  trajectory_msgs::JointTrajectoryPoint to_pt;
  to_pt.time_from_start = ros::Duration(max_t);
  for (unsigned i = 0; i < seedv.size(); ++i)
    to_pt.positions.push_back(result(i));
  traj.points.push_back(to_pt);

  goal.trajectory = traj;

  auto t4 = std::chrono::steady_clock::now();
  ac_.sendGoal(goal);
  ac_.waitForResult();
  auto t5 = std::chrono::steady_clock::now();

  using namespace std::chrono;
  long dt_ik = duration_cast<milliseconds>(t3 - t2).count();
  long dt_goal = duration_cast<milliseconds>(t5 - t4).count();

//  ROS_INFO("Capture/Ik/Goal: %ld %ld", dt_ik, dt_goal);

  return true;
}
