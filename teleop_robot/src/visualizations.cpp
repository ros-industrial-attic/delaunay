#include "visualizations.hpp"

const std::string marker_ns = "ns";
const int source_id = 0;
const int latch_id = 1;

double scale = 0.005;

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

