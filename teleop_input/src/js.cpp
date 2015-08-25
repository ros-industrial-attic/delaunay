#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <boost/ref.hpp>

const double MAX_X_VEL = 0.003;
const double MAX_Y_VEL = 0.003;
const double MAX_Z_VEL = 0.003;
const double MAX_ANGULAR_Y_POS = M_PI / 8.0;
const double MAX_ANGULAR_X_POS = M_PI / 8.0;
const double BUTTON_LINEAR_DELTA_Z = 0.005;
const double BUTTON_ANGULAR_DELTA_Z = 0.01;

const double MAX_ANGULAR_Z_VEL = 0.01;

double mapX(double ratio)
{
  return ratio * -MAX_X_VEL;
}

double mapY(double ratio)
{
  return ratio * MAX_X_VEL;
}

double mapZ(double ratio)
{
  return ratio * MAX_Z_VEL;
}

double mapAngularZ(double ratio)
{
  return ratio * MAX_ANGULAR_Z_VEL;
}

double mapAngularY(double ratio)
{
  return ratio * MAX_ANGULAR_Y_POS;
}

double mapAngularX(double ratio)
{
  return ratio * MAX_ANGULAR_X_POS;
}

double buttonLinearZ(bool v) { return v ? BUTTON_LINEAR_DELTA_Z : 0.0; }
double buttonAngularZ(bool v) { return v ? BUTTON_ANGULAR_DELTA_Z : 0.0; }


geometry_msgs::Twist interpret(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.x = mapX(joy.axes[0]);
  twist.linear.y = mapY(joy.axes[1]);
  twist.linear.z = buttonLinearZ(joy.buttons[4]) - buttonLinearZ(joy.buttons[6]);

  twist.angular.x = mapAngularX(-joy.axes[3]);
  twist.angular.y = mapAngularY(-joy.axes[2]);
  twist.angular.z = buttonAngularZ(joy.buttons[7]) - buttonAngularZ(joy.buttons[5]);

  return twist;
}

bool isInactive(double v) { return std::abs(v) < 0.0001; }


void handleSixAxisPS3(const sensor_msgs::JoyConstPtr& joy, 
                      ros::Publisher& output_pub,
                      ros::Publisher& mode_pub)
{
  static bool select_button_last_state = false;
  if (joy->buttons[0] != select_button_last_state)
  {
    if (joy->buttons[0]) {
      ROS_INFO("Rising edge... publishing");
      std_msgs::Empty empty;
      mode_pub.publish(empty);
    }
    select_button_last_state = joy->buttons[0];
  }

  if (isInactive(joy->axes[0]) && isInactive(joy->axes[1]) && 
      isInactive(joy->axes[2]) && isInactive(joy->axes[3]) &&
      isInactive(joy->axes[12]) && isInactive(joy->axes[13]) &&
      !joy->buttons[0] && !joy->buttons[4] && !joy->buttons[5] &&
      !joy->buttons[6] && !joy->buttons[7]

      ) return;

  geometry_msgs::Twist out = interpret(*joy);

  output_pub.publish(out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop_node");
  
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("commands", 1);
  ros::Publisher mode_pub = nh.advertise<std_msgs::Empty>("change_mode", 1);
  
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, 
    boost::bind(handleSixAxisPS3, _1, boost::ref(pub), boost::ref(mode_pub))); 

  ros::spin();

  return 0;
}
