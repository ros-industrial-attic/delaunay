#include "teleop_robot/robot_interface.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <chrono>

#include "trajectory_msgs/JointTrajectory.h"

teleop_robot::RobotInterface::RobotInterface(const std::string& group)
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
                                               const Eigen::Affine3d& target_pose,
                                               ros::Publisher& pub)
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
  sensor_msgs::JointState state;
  state.header.frame_id = "world";
  state.header.stamp = ros::Time::now();
  state.name = joint_names_;
  for (unsigned i = 0; i < seedv.size(); ++i)
    state.position.push_back(result(i));

  pub.publish(state);

  return true;
}

bool teleop_robot::RobotInterface::fk(const Eigen::VectorXd& joints, Eigen::Affine3d& pose) const
{
  return kin_.calcFwdKin(joints, pose);
}

bool teleop_robot::RobotInterface::ik(const Eigen::Affine3d& pose, const Eigen::VectorXd& seed,
                                      Eigen::VectorXd& solution) const
{
  try {
    solver_.calcInvKin(pose, seed, planning_scene_, solution);
    return true;
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
}



