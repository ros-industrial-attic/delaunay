#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <boost/ref.hpp>
#include <visualization_msgs/Marker.h>
#include "eigen3/Eigen/Dense"

const std::string marker_ns = "ns";
const int mesh_id = 3;

visualization_msgs::Marker makeMarker(const Eigen::Vector3d& pt, const std::string& meshname)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_ns;
  marker.id = mesh_id;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.pose.position.x = pt(0);
  marker.pose.position.y = pt(1);
  marker.pose.position.z = pt(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.2f;
  marker.color.g = 0.2f;
  marker.color.b = 0.2f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker.mesh_resource = "file://" + meshname;
  return marker;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_tracking_test");
  ros::NodeHandle pnh ("~");

  std::string mesh_file;
  pnh.param<std::string>("mesh_file", mesh_file, "");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("markers", 1000);

  ros::Rate rate (0.5);

  while (ros::ok())
  {
    ros::spinOnce();
    pub.publish(makeMarker(Eigen::Vector3d(0,0,0), mesh_file));
    rate.sleep();
  }

  return 0;
}

