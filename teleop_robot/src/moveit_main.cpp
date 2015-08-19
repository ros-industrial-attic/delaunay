﻿#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <boost/ref.hpp>

#include "teleop_tracking/mesh.h"
#include "visualizations.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "teleop_robot/robot_interface.h"

struct TeleopOptions
{
  double max_theta; // deviation from world z (rad)
  double standoff; // distance (m)
};

class Teleop
{
public:
  Teleop(teleop_tracking::Mesh& mesh,
         teleop_robot::RobotInterface& interface,
         moveit::planning_interface::MoveGroup& group,
         const TeleopOptions& options,
         ros::Publisher& pose_pub)
    : mesh_(mesh)
    , interface_(interface)
    , group_(group)
    , options_(options)
    , pose_pub_(pose_pub)
  {
    triangle_pose_ = mesh_.closestPose(Eigen::Vector3d(0.05, 0.05, 1));
  }

  void update(const geometry_msgs::TwistConstPtr& mv)
  {
    boost::mutex::scoped_lock lock(planning_mutex_, boost::try_to_lock);
    if (!lock.owns_lock())
    {
      ROS_WARN("WAIT YOUR TURN");
      return;
    }
    ROS_INFO_STREAM("UPDATE\n" << *mv);
    group_.setStartStateToCurrentState();
    // TODO: Make this all thread safe
    // calculate new pose
    Eigen::Affine3d new_pose = triangle_pose_.pose;
    unsigned new_index = triangle_pose_.index;

    if (mv->linear.z != 0.0)
    {
      options_.standoff += mv->linear.z;
      ROS_INFO("NEW STANDOFF: %f", options_.standoff);
      // new_pose = triangle_pose_.pose;
      // new_index = triangle_pose_.index;
    }

    if (mv->angular.z != 0.0)
    {

      Eigen::AngleAxisd z_rot (mv->angular.z, Eigen::Vector3d::UnitZ());
      new_pose = new_pose * z_rot;
      ROS_INFO("ROTATING");
    }
    
    if (std::abs(mv->linear.x) > 0.0001 || std::abs(mv->linear.y) > 0.0001)
    {
      Eigen::Vector2d t (mv->linear.x, mv->linear.y);
      new_pose =  mesh_.walkTriangle2(new_pose,
                                      new_index,
                                      new_index,
                                      t);
      ROS_INFO("WALKING");
    }

    // Publish preview
    pose_pub_.publish(makeStampedPose(new_pose));

    // plan and execute
    Eigen::Affine3d target_pose = modelToRobotPose(Eigen::Translation3d(0.5, 0, 0)*new_pose);

    if (seed_.empty())
    {
      ROS_INFO("WAITING");
      return;
    }

    if (!interface_.planAndMove(seed_, target_pose))
    {
      return;
    }


    triangle_pose_.index = new_index;
    triangle_pose_.pose = new_pose;
  }

  void updateSeedState(const sensor_msgs::JointStateConstPtr& joint_ptr)
  {
    seed_ = joint_ptr->position;
  }

private:
  Eigen::Affine3d modelToRobotPose(const Eigen::Affine3d& model) const
  {
    const static Eigen::AngleAxisd flip_z (M_PI, Eigen::Vector3d::UnitX());
    Eigen::Affine3d target;
    target = model * Eigen::Translation3d(0, 0, options_.standoff) * flip_z;

    // Calculate offset between world and pose

    const Eigen::Vector3d& model_z = model.matrix().col(2).head<3>();
    const Eigen::Vector3d& world_z = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axis = model_z.cross(world_z);
    double amt = std::acos(model_z.dot(world_z));

    ROS_INFO_STREAM("Rot:\n" << axis << "\namt: " << amt);

    if (std::abs(amt) > options_.max_theta)
    {
      Eigen::Vector3d local_axis = model.inverse().linear() * axis;
      Eigen::AngleAxisd local_rot(options_.max_theta, local_axis.normalized());
      target = model * local_rot * Eigen::Translation3d(0, 0, options_.standoff) * flip_z;

    }

    return target;
  }

  bool executePose(const Eigen::Affine3d& target)
  {
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(target, p);

    group_.setPoseTarget(p);
    moveit::planning_interface::MoveGroup::Plan plan;
    ROS_INFO("PLANNING...");
    if (!group_.plan(plan))
    {
      ROS_WARN_STREAM("Could not plan to pose:\n" << target.matrix());
      return false;
    }
    ROS_INFO("EXECUTING...");
    if (!group_.execute(plan))
    {
      ROS_WARN_STREAM("Could not execute plan to pose:\n" << target.matrix());
      return false;
    }
    ROS_INFO("DONE...");
    return true;
  }

  // Current state
  teleop_tracking::TrianglePose triangle_pose_;
  TeleopOptions options_;
  boost::mutex planning_mutex_;

  // Member objects
  teleop_tracking::Mesh& mesh_;
  teleop_robot::RobotInterface& interface_;
  moveit::planning_interface::MoveGroup& group_;
  ros::Publisher& pose_pub_;
  std::vector<double> seed_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot_node");
//  ros::AsyncSpinner spinner (2);
//  spinner.start();

  ros::NodeHandle nh, pnh("~");

  std::string mesh_name;
  pnh.param<std::string>("mesh_file", mesh_name, "");

  if (mesh_name.empty())
  {
    ROS_FATAL("Mesh name required");
    return 1;
  }

  teleop_tracking::Mesh mesh(mesh_name);
  // config group
  moveit::planning_interface::MoveGroup group ("manipulator_camera");
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(3.0);

  TeleopOptions options;
  options.max_theta = 0.8;
  options.standoff = 0.25;

  teleop_robot::RobotInterface interface("manipulator_camera");

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

  Teleop teleop (mesh, interface, group, options, pub);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("commands", 1, boost::bind(&Teleop::update, &teleop, _1));
  ros::Subscriber state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(&Teleop::updateSeedState, &teleop, _1));

  ros::spin();

  return 0;
}