#include "ros/ros.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Trail
{
  Trail()
    : cloud(new PointCloud)
  {}

  PointCloud::Ptr cloud;
};

void handleNewPoint(const geometry_msgs::Point::ConstPtr& pt,
                    Trail& trail)
{
  trail.cloud->points.push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
}

void publishTrail(const ros::TimerEvent&, ros::Publisher& pub, const Trail& trail)
{
  pub.publish(trail.cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trail_publisher");
  ros::NodeHandle nh, pnh("~");

  std::string frame;
  nh.param<std::string>("frame", frame, "world");

  double period;
  nh.param<double>("period", period, 1.0);

  Trail trail;
  trail.cloud->header.frame_id = frame;

  ros::Subscriber sub =
      nh.subscribe<geometry_msgs::Point>("incident_point", 100,
                                         boost::bind(handleNewPoint, _1, boost::ref(trail)));

  ros::Publisher pub =
      nh.advertise<PointCloud>("trail", 1);

  ros::Timer timer = nh.createTimer(ros::Duration(period),
                                    boost::bind(publishTrail,
                                                _1,
                                                boost::ref(pub),
                                                boost::cref(trail)));

  ros::spin();
}
