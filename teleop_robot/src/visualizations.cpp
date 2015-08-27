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


visualization_msgs::Marker makePointsMarker(const std::vector<geometry_msgs::Point> &pts)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "painting";
  marker.id = 11;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
//  marker.scale.z = 1;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.points = pts;
  marker.lifetime = ros::Duration();
  return marker;
}
