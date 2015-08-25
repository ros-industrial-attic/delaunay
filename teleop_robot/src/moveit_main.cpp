#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
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

/**
 * 
 */
class TrackingMode {
public:
  struct Transaction {
    Eigen::Affine3d target; 
    teleop_tracking::TrianglePose triangle_pose;
  };

  TrackingMode(const teleop_tracking::Mesh& mesh, const TeleopOptions& options)
    : mesh_(mesh)
    , options_(options)
  {}

  void start(const Eigen::Affine3d& init_pose) 
  {
    // closest point start
    current_pose_ = mesh_.closestPose(init_pose.translation() - Eigen::Vector3d(0.5, 0, 0)); 
  }

  void startDir(const Eigen::Affine3d& init_pose) 
  {    
    teleop_tracking::TrianglePosition intersect;
    double dist;
    if (mesh_.intersectRay(init_pose.translation() - Eigen::Vector3d(0.5, 0, 0),
                           init_pose.matrix().col(2).head<3>(),
                           intersect, dist))
    {
      current_pose_ = mesh_.toPose(intersect);
    }
    else
    {
      ROS_WARN("Could not intersect point-of-view with part");
    }
  }

  Transaction calc(const geometry_msgs::Twist& twist)
  {
    // calculate new pose
    Transaction transaction;
    transaction.triangle_pose = current_pose_;

    Eigen::Affine3d& new_pose = transaction.triangle_pose.pose;
    unsigned& new_index = transaction.triangle_pose.index;


    if (twist.linear.z != 0.0)
    {
       options_.standoff += twist.linear.z;
    }

    if (twist.angular.z != 0.0)
    {
      Eigen::AngleAxisd z_rot (twist.angular.z, Eigen::Vector3d::UnitZ());
      new_pose = new_pose * z_rot;
    }

    
    if (std::abs(twist.linear.x) > 0.0001 || std::abs(twist.linear.y) > 0.0001)
    {
      Eigen::Vector2d t (twist.linear.x, twist.linear.y);
      new_pose =  mesh_.walkTriangle2(new_pose, new_index, new_index, t);
    }

    transaction.target = modelToRobotPose(Eigen::Translation3d(0.5, 0, 0)*new_pose,
                                          twist.angular);
    return transaction;
  }

  void update(const Transaction& transaction)
  {
    current_pose_ = transaction.triangle_pose;
  }

  const teleop_tracking::Mesh& mesh() const { return mesh_; }

private:
  Eigen::Affine3d modelToRobotPose(const Eigen::Affine3d& model,
                                   const geometry_msgs::Vector3& ang) const
  {
    const static Eigen::AngleAxisd flip_z (M_PI, Eigen::Vector3d::UnitX());


    Eigen::Affine3d target;
    target = model;

    // Calculate offset between world and pose

    const Eigen::Vector3d& model_z = model.matrix().col(2).head<3>();
    const Eigen::Vector3d& world_z = Eigen::Vector3d::UnitZ();
    double amt = std::acos(model_z.dot(world_z));

    // Apply the static limits to tool orientation from Z
    if (std::abs(amt) > options_.max_theta)
    {
      Eigen::Vector3d axis = model_z.cross(world_z);
      Eigen::Vector3d local_axis = model.inverse().linear() * axis;
      Eigen::AngleAxisd local_rot(options_.max_theta, local_axis.normalized());
      target = model * local_rot;
    }

    // If the user gave x or y angular input, perform further adjustments
    if (ang.x != 0.0)
    {
      target *= Eigen::AngleAxisd(ang.x, Eigen::Vector3d::UnitX());
    }

    if (ang.y != 0.0)
    {
      target *= Eigen::AngleAxisd(ang.y, Eigen::Vector3d::UnitY());
    }

    // Translate to a valid camera position
     target = target * Eigen::Translation3d(0, 0, options_.standoff) * flip_z;

    return target;
  }
  
  const teleop_tracking::Mesh& mesh_;
  teleop_tracking::TrianglePose current_pose_;
  TeleopOptions options_;
};

/**
 * 
 */
class FreeMode {
public:
  struct Transaction {
    Eigen::Affine3d target;
  };

  FreeMode()
    : current_pose_(Eigen::Affine3d::Identity()) 
  {}

  void start(const Eigen::Affine3d& init_pose)
  {
    current_pose_ = init_pose;
  }

  Transaction calc(const geometry_msgs::Twist& twist) const
  {
    Transaction trans;
    trans.target = current_pose_;
    Eigen::Affine3d step;
    step = Eigen::Translation3d(twist.linear.x, -twist.linear.y, -twist.linear.z);

    step = step * Eigen::AngleAxisd(twist.angular.x/10.0, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(-twist.angular.y/10.0, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(twist.angular.z, Eigen::Vector3d::UnitZ());

    trans.target = trans.target * step;
    return trans;
  }

  void update(const Transaction& transaction)
  {
    current_pose_ = transaction.target;
  }

private:
  Eigen::Affine3d current_pose_;
};


class Teleop
{
public:
  enum class OperationMode
  {
    Tracking, Free
  };

  Teleop(teleop_tracking::Mesh& mesh,
         teleop_robot::RobotInterface& interface,
         const TeleopOptions& options,
         ros::Publisher& pose_pub,
         ros::Publisher& state_pub,
         ros::Publisher& visual_pub)
    : interface_(interface)
    , pose_pub_(pose_pub)
    , state_pub_(state_pub)
    , visual_pub_(visual_pub)
    , last_message_(ros::Time::now())
    , tracking_mode_(mesh, options)
    , free_mode_()
    , active_mode_(OperationMode::Tracking)
  {
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    start_pose.translation() = Eigen::Vector3d(0.1, 0.1, 1.0);
    tracking_mode_.start(start_pose);
  }

  void update(const geometry_msgs::TwistConstPtr& mv)
  {
    if (seed_.empty())
    {
      return;
    }

    if (ros::Time::now() - last_message_ < ros::Duration(1.0 / 30.0))
    {
      return;
    }
    last_message_ = ros::Time::now();

    if (active_mode_ == OperationMode::Tracking) 
    {
      // tracking surface
      TrackingMode::Transaction t = tracking_mode_.calc(*mv);
      pose_pub_.publish(makeStampedPose(t.triangle_pose.pose));
      if (interface_.planAndMove(seed_, t.target, state_pub_))
      {
        tracking_mode_.update(t); 
      }
      else
      {
        ROS_WARN("Failed to move to new track position.");
      }
    } 
    else 
    {
      // free move
      FreeMode::Transaction t = free_mode_.calc(*mv);
      if (interface_.planAndMove(seed_, t.target, state_pub_))
      {
        free_mode_.update(t);
        // cast ray
        teleop_tracking::TrianglePosition intersect;
        double dist;
        if (tracking_mode_.mesh().intersectRay(t.target.translation() - Eigen::Vector3d(0.5, 0, 0),
                               t.target.matrix().col(2).head<3>(),
                               intersect, dist))
        {
          visual_pub_.publish(makeMarker(intersect.position, 0));
        }

      }
      else
      {
        ROS_WARN("Failed to move to new free position");
      }
    }
  }

  void updateSeedState(const sensor_msgs::JointStateConstPtr& joint_ptr)
  {
    seed_ = joint_ptr->position;
  }

  void updateMode(OperationMode mode)
  {
    if (mode == active_mode_) return;

    Eigen::Affine3d eef_pose;
    if (!interface_.fk(teleop_robot::fromVector(seed_), eef_pose))
    {
      ROS_WARN("Failed to compute IK to seed track mode.");
      return;
    }
    active_mode_ = mode;
    
    if (mode == OperationMode::Tracking) 
    {
      tracking_mode_.startDir(eef_pose);
    }
    else
    {
      free_mode_.start(eef_pose);
    }
  }
  
  OperationMode mode() const { return active_mode_; } 

private:
  // Motion modes
  TrackingMode tracking_mode_;
  FreeMode free_mode_;
  OperationMode active_mode_;

  // Current state
  ros::Time last_message_;

  // Member objects
  teleop_robot::RobotInterface& interface_;
  ros::Publisher& pose_pub_;
  ros::Publisher& state_pub_;
  ros::Publisher& visual_pub_;
  std::vector<double> seed_;
};

void callback(const std_msgs::EmptyConstPtr&, Teleop& teleop)
{
  ROS_INFO("TOGGLING MODE");
  if (teleop.mode() == Teleop::OperationMode::Free) {
    teleop.updateMode(Teleop::OperationMode::Tracking);
  } else {
    teleop.updateMode(Teleop::OperationMode::Free);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot_node");
  ros::NodeHandle nh, pnh("~");

  std::string mesh_name;
  pnh.param<std::string>("mesh_file", mesh_name, "");

  if (mesh_name.empty())
  {
    ROS_FATAL("Mesh name required");
    return 1;
  }

  teleop_tracking::Mesh mesh(mesh_name);

  TeleopOptions options;
  options.max_theta = M_PI_4;
  options.standoff = 0.25;

  teleop_robot::RobotInterface interface("manipulator_camera");

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  ros::Publisher state_pub = nh.advertise<sensor_msgs::JointState>("target", 1);
  ros::Publisher visual_pub = nh.advertise<visualization_msgs::Marker>("looking", 1);
  Teleop teleop (mesh, interface, options, pub, state_pub, visual_pub);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("commands", 1, boost::bind(&Teleop::update, &teleop, _1));
  ros::Subscriber state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(&Teleop::updateSeedState, &teleop, _1));
  ros::Subscriber mode_sub = nh.subscribe<std_msgs::Empty>("change_mode", 1, boost::bind(callback, _1, boost::ref(teleop)));

  ros::spin();

  return 0;
}
