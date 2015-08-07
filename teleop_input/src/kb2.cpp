#include <ros/ros.h>
#include <geometry_msgs/Point.h>
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
   geometry_msgs::Point pt_;
   ros::Publisher vel_pub_;

 };

 TeleopTurtle::TeleopTurtle()
 {
   vel_pub_ = nh_.advertise<geometry_msgs::Point>("points", 1);
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
         pt_.x = step;
         dirty = true;
         break;
       case KEYCODE_R:
         ROS_DEBUG("RIGHT");
         pt_.x = -step;
         dirty = true;
         break;
       case KEYCODE_U:
         ROS_DEBUG("UP");
         pt_.z = step;
         dirty = true;
         break;
       case KEYCODE_D:
         ROS_DEBUG("DOWN");
         pt_.z = -step;
         dirty = true;
         break;
       case 'a':
         pt_.y = step;
         dirty = true;
         break;
       case 'd':
         pt_.y = -step;
         dirty = true;
       break;
       case 't':
         step += 0.0005;
       break;
     case 'y':
       step -= 0.0005;
       break;
     case 'o':
       pt_.y = step;
       pt_.x = step;
       dirty = true;
       break;
     case 'p':
       pt_.y = -step;
       pt_.x = -step;
       dirty = true;
       break;
     }


     if(dirty ==true)
     {
       vel_pub_.publish(pt_);
       pt_.x = 0.0;
       pt_.y = 0.0;
       pt_.z = 0.0;
       dirty=false;
     }
   }


   return;
 }

