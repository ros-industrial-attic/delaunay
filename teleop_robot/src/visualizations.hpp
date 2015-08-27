#ifndef VISUALIZATIONS_HPP
#define VISUALIZATIONS_HPP

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

visualization_msgs::Marker makeMarker(const Eigen::Vector3d& pt, int marker_id);

visualization_msgs::Marker makePointsMarker(const std::vector<geometry_msgs::Point>& pts);

geometry_msgs::PoseStamped makeStampedPose(const Eigen::Affine3d& pose);

#endif // VISUALIZATIONS_HPP

