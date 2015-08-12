
#include "teleop_tracking/mesh.h"

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <boost/ref.hpp>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

const std::string marker_ns = "ns";
const int source_id = 0;
const int latch_id = 1;

double scale = 0.1;

visualization_msgs::Marker makeMarker(const Eigen::Vector3d& pt, int marker_id)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = pt(0);
  marker.pose.position.y = pt(1);
  marker.pose.position.z = pt(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = (marker_id == source_id ? 0.0f : 1.0f);
  marker.color.g = (marker_id == source_id ? 1.0f : 0.0f);
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  return marker;
}

geometry_msgs::PoseStamped makeStampedPose(const Eigen::Affine3d& pose)
{
  geometry_msgs::PoseStamped stamped;
  stamped.header.frame_id = "world";
  stamped.header.stamp = ros::Time::now();
  tf::poseEigenToMsg(pose, stamped.pose);
  return stamped;
}


void pointCallback(const geometry_msgs::Point::ConstPtr& pt,
                   const teleop_tracking::Mesh& mesh,
                   ros::Publisher& pub,
                   ros::Publisher& pose_pub)
{
  using teleop_tracking::TrianglePose;
  using teleop_tracking::TrianglePosition;

  static TrianglePose current_tpose = mesh.closestPose(Eigen::Vector3d(0.05,0.1,0.5));

  ROS_WARN("Moving %f %f %f", pt->x, pt->y, pt->z);

  if (pt->z != 0.0)
  {
    current_tpose = mesh.closestPose(current_tpose.pose.translation());
    geometry_msgs::PoseStamped gpose = makeStampedPose(current_tpose.pose);
    std::cout << current_tpose.pose.matrix() << '\n';
    if (current_tpose.pose.matrix()(0,0) != current_tpose.pose.matrix()(0,0)) exit(1);
    pose_pub.publish(gpose);
    return;
  }

  Eigen::Vector3d v (pt->x, pt->y, pt->z);


  Eigen::Affine3d new_pose = mesh.walkTriangle2(current_tpose.pose,
                                                current_tpose.index,
                                                current_tpose.index,
                                                Eigen::Vector2d(v(0), v(1)));

  std::cout << "Current Indx: " << current_tpose.index << "\n";

  geometry_msgs::PoseStamped gpose1 = makeStampedPose(current_tpose.pose);

  current_tpose.pose = new_pose;

  Eigen::Vector3d int_point = new_pose.translation();
  visualization_msgs::Marker m_pt = makeMarker(int_point, latch_id);


  pub.publish(m_pt);
  pose_pub.publish(gpose1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_tracking_test");
  ros::NodeHandle pnh ("~");

  std::string mesh_file;
  pnh.param<std::string>("mesh_file", mesh_file, "");

  pnh.param<double>("scale", scale, 0.1);

  if (mesh_file.empty())
  {
    ROS_FATAL("Mesh Tracking Test requires the 'mesh_file' param be set.");
    return -1;
  }

  std::cout << "Loading mesh...\n";

  teleop_tracking::Mesh mesh(mesh_file);

  std::cout << "Done loading mesh\n";

  mesh.debugInfo();

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("markers", 1000);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("poses", 1000);
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Point>("points", 1, boost::bind(pointCallback, _1, boost::cref(mesh), boost::ref(pub), boost::ref(pose_pub)));

  ros::spin();

  return 0;
}
