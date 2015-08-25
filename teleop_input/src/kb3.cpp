#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

 class TeleopTurtle
 {
 public:
   TeleopTurtle();
   void keyLoop();

 private:


   ros::NodeHandle nh_;
   geometry_msgs::Twist pt_;
   ros::Publisher vel_pub_;

 };

 TeleopTurtle::TeleopTurtle()
 {
   vel_pub_ = nh_.advertise<geometry_msgs::Twist>("commands", 1);
 }

 int kfd = 0;
 struct termios cooked, raw;

 void quit(int sig)
 {
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
 }


 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "teleop_turtle");
   TeleopTurtle teleop_turtle;

   signal(SIGINT,quit);

   teleop_turtle.keyLoop();

   return(0);
 }


 void TeleopTurtle::keyLoop()
 {
   char c;
   bool dirty=false;
   double step = 0.002;


   // get the console in raw mode
   tcgetattr(kfd, &cooked);
   memcpy(&raw, &cooked, sizeof(struct termios));
   raw.c_lflag &=~ (ICANON | ECHO);
   // Setting a new line, then end of file
   raw.c_cc[VEOL] = 1;
   raw.c_cc[VEOF] = 2;
   tcsetattr(kfd, TCSANOW, &raw);

   puts("Reading from keyboard");
   puts("---------------------------");
   puts("Use arrow keys to move the turtle.");


   for(;;)
   {
     // get the next event from the keyboard
     if(read(kfd, &c, 1) < 0)
     {
       perror("read():");
       exit(-1);
     }

     ROS_DEBUG("value: 0x%02X\n", c);

     switch(c)
     {
       case KEYCODE_L:
         ROS_DEBUG("LEFT");
         pt_.linear.x = step;
         dirty = true;
         break;
       case KEYCODE_R:
         ROS_DEBUG("RIGHT");
         pt_.linear.x = -step;
         dirty = true;
         break;
       case KEYCODE_U:
         ROS_DEBUG("UP");
         pt_.linear.z = step;
         dirty = true;
         break;
       case KEYCODE_D:
         ROS_DEBUG("DOWN");
         pt_.linear.z = -step;
         dirty = true;
         break;
       case 'a':
         pt_.linear.y = step;
         dirty = true;
         break;
       case 'd':
         pt_.linear.y = -step;
         dirty = true;
       break;
       case 't':
         step += 0.0005;
         ROS_INFO("step size: %f", step);
       break;
       case 'y':
         step -= 0.0005;
         ROS_INFO("step size: %f", step);
         break;
       case 'q':
          pt_.angular.z = 0.01;
          dirty = true;
          break;
       case 'e':
          pt_.angular.z = -0.01;
          dirty = true;
          break;
       case 'o':
          pt_.angular.x = 0.2;
          dirty = true;
          break;
       case 'p':
          pt_.angular.x = -0.2;
          dirty = true;
          break;
       case 'k':
          pt_.angular.y = 0.2;
          dirty = true;
          break;
       case 'l':
          pt_.angular.y = -0.2;
          dirty = true;
          break;
     }


     if(dirty ==true)
     {
       vel_pub_.publish(pt_);
       pt_.linear.x = 0.0;
       pt_.linear.y = 0.0;
       pt_.linear.z = 0.0;
       pt_.angular.x = 0.0;
       pt_.angular.y = 0.0;
       pt_.angular.z = 0.0;
       dirty=false;
     }
   }


   return;
 }

