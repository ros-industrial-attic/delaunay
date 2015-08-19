#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/ref.hpp>

const double MAX_X_VEL = 0.003;
const double MAX_Y_VEL = 0.003;

const double MAX_ANGULAR_Z_VEL = 0.001;

double mapX(double ratio)
{
  return ratio * -MAX_X_VEL;
}

double mapY(double ratio)
{
  return ratio * MAX_X_VEL;
}

double mapAngularZ(double ratio)
{
  return ratio * MAX_ANGULAR_Z_VEL;
}

geometry_msgs::Twist interpret(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.x = mapX(joy.axes[0]);
  twist.linear.y = mapY(joy.axes[1]);

  twist.angular.z = mapAngularZ(joy.axes[2]);

  return twist;
}

void handleSixAxisPS3(const sensor_msgs::JoyConstPtr& joy, ros::Publisher& output_pub)
{
  if (std::abs(joy->axes[0]) < 0.00001 && 
      std::abs(joy->axes[1]) < 0.00001 &&
      std::abs(joy->axes[2]) < 0.00001) return;

  geometry_msgs::Twist out = interpret(*joy);

  output_pub.publish(out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop_node");
  
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("commands", 1); 
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(handleSixAxisPS3, _1, boost::ref(pub))); 

  ros::spin();

  return 0;
}
