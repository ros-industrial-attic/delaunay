#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

// Special Keycode Constants
#define KEYCODE_R 0x43 // arrow key right
#define KEYCODE_L 0x44 // arrow key left
#define KEYCODE_U 0x41 // arrow key up
#define KEYCODE_D 0x42 // arrow key down
#define KEYCODE_Q 0x71 // escape?

// Topic names
#define TWIST_MSG_TOPIC "commands"
#define MODE_MSG_TOPIC "change_mode"

 class TeleopTurtle
 {
 public:
   TeleopTurtle();
   void keyLoop();

 private:


   ros::NodeHandle nh_;
   geometry_msgs::Twist pt_;
   ros::Publisher vel_pub_;
   ros::Publisher mode_pub_;
 };

 TeleopTurtle::TeleopTurtle()
 {
   vel_pub_ = nh_.advertise<geometry_msgs::Twist>(TWIST_MSG_TOPIC, 1);
   mode_pub_ = nh_.advertise<std_msgs::Empty>(MODE_MSG_TOPIC, 1);
 }

 int kfd = 0;
 struct termios cooked, raw;
 bool global_abort_requested = false;

 void quit(int sig)
 {
   tcsetattr(kfd, TCSANOW, &cooked);
   global_abort_requested = true;
 }


 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "teleop_turtle");
   TeleopTurtle teleop_turtle;

   signal(SIGINT, quit);

   teleop_turtle.keyLoop();

   return 0;
 }


 void TeleopTurtle::keyLoop()
 {
   char c;
   bool dirty = false;
   bool mode_dirty = false;

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
   puts("Use W, A, S, D to move in cartesian directions.");
   puts("Use Z and C to move in and out.");
   puts("Use Q and E to yaw the camera.");
   puts("Use arrow keys to adjust view angle.");
   puts("Use SPACE to toggle motion mode.");
   puts("Use T and Y to adjust the step size.");


   while(!global_abort_requested)
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
       // Cartesian Motion
       case 'd':
         pt_.linear.x = step;
         dirty = true;
         break;
       case 'a':
         pt_.linear.x = -step;
         dirty = true;
         break;
       case 'c':
         pt_.linear.z = step;
         dirty = true;
         break;
       case 'z':
         pt_.linear.z = -step;
         dirty = true;
         break;
       case 'w':
         pt_.linear.y = step;
         dirty = true;
         break;
       case 's':
         pt_.linear.y = -step;
         dirty = true;
         break;
       // Adjust step sizes
       case 't':
         step += 0.0005;
         ROS_INFO("step size: %f", step);
         break;
       case 'y':
         step -= 0.0005;
         ROS_INFO("step size: %f", step);
         break;
         // Rotations
       case 'q':
          pt_.angular.z = 0.01;
          dirty = true;
          break;
       case 'e':
          pt_.angular.z = -0.01;
          dirty = true;
          break;
       case KEYCODE_D:
          pt_.angular.x = 0.2;
          dirty = true;
          break;
       case KEYCODE_U:
          pt_.angular.x = -0.2;
          dirty = true;
          break;
       case KEYCODE_R:
          pt_.angular.y = 0.2;
          dirty = true;
          break;
       case KEYCODE_L:
          pt_.angular.y = -0.2;
          dirty = true;
          break;
       // Change mode
       case ' ':
          mode_dirty = true;
          break;
     }


     if (dirty)
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

     if (mode_dirty)
     {
       std_msgs::Empty empty;
       mode_pub_.publish(empty);
       mode_dirty = false;
     }

   } // end main loop


   return;
 }

