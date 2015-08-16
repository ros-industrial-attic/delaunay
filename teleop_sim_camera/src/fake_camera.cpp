#include <ros/ros.h>
// Image publishing
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// camera info drop
#include <sensor_msgs/CameraInfo.h>

struct FakeCameraParams
{
  std::string topic;
  std::string frame;
  int width, height;
  double rate;
};

FakeCameraParams loadParams(ros::NodeHandle& nh)
{
  FakeCameraParams params;
  params.frame = nh.param<std::string>("frame", "camera");
  params.width = nh.param<int>("image_width", 640);
  params.height = nh.param<int>("image_height", 480);
  params.rate = nh.param<double>("rate", 30.0);
  params.topic = nh.param<std::string>("topic", "camera");
  return params;
}

sensor_msgs::CameraInfo loadCameraInfo(const FakeCameraParams& params)
{
  sensor_msgs::CameraInfo info;
  info.header.frame_id = params.frame;
  info.header.stamp = ros::Time::now();

  info.height = params.height;
  info.width = params.width;
  info.distortion_model = "plumb_bob";
  info.D = std::vector<double>(5, 0.0);

  double focal_length = 360.0;
  info.K = {
    focal_length, 0, params.width/2.0,
    0, focal_length, params.height/2.0,
    0, 0, 1
  };

  info.R = {1, 0, 0,
            0, 1, 0,
            0, 0, 1};

  info.P = {
    focal_length, 0, params.width/2.0, 0,
    0, focal_length, params.height/2.0, 0,
    0, 0, 1, 0
  };

  return info;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh, pnh("~");

  FakeCameraParams params = loadParams(pnh);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub =
      it.advertise(params.topic+"/image", 1);

  ros::Publisher info_pub =
      nh.advertise<sensor_msgs::CameraInfo>(params.topic+"/camera_info", 1);
  sensor_msgs::CameraInfo info = loadCameraInfo(params);

  cv::Mat image =
      cv::Mat(params.width, params.height, CV_8UC3, cv::Scalar(50,50,50));
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  msg->header.frame_id = params.frame;
  msg->header.stamp = ros::Time::now();

  ros::Rate loop_rate(params.rate);

  while (nh.ok()) {
    msg->header.stamp = ros::Time::now();
    info.header.stamp = ros::Time::now();
    pub.publish(msg);
    info_pub.publish(info);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
