#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

static void interpolateJoints(const std::vector<double>& start,
                              const std::vector<double>& stop,
                              double ratio,
                              std::vector<double>& out)
{
  for (std::size_t i = 0; i < start.size(); ++i) {
    out[i] = start[i] + (stop[i] - start[i]) * ratio;
  }
}

class RobotSimulator
{
public:
  RobotSimulator(double rate, const std::vector<std::string>& names)
    : target_position_(names.size(), 0.0)
    , joint_velocities_(names.size(), 1.0)
    , joint_names_(names)
    , pub_rate_(rate)
    , active_flag_(false)
  {
  }

  ~RobotSimulator()
  {
    stop();
  }

  void start()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (active_flag_) throw std::runtime_error("Don't call start twice");
    active_flag_ = true;

    worker_ = boost::thread(&RobotSimulator::worker_main, this);
  }

  void stop()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    active_flag_ = false;
  }

  void update(const sensor_msgs::JointState& state) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    target_position_ = state.position;
  }

private:
  void worker_main()
  {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    // State
    std::vector<double> current_position (joint_names_.size(), 0.0);
    ros::Rate rate (pub_rate_);

    // Prep message
    sensor_msgs::JointState state;
    state.header.frame_id = "world";
    state.header.stamp = ros::Time::now();
    state.name = joint_names_;

    // Main
    while (true)
    {
      // Fetch target
      std::vector<double> target;
      {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!active_flag_) break;
        target = target_position_;
      }

      // Calculate maximum step
      interpolateJoints(current_position, target_position_, 0.03, current_position);

      // update message and publish
      state.position = current_position;
      state.header.stamp = ros::Time::now();
      pub.publish(state);

      // sleep
      rate.sleep();
    }
  }

  // State data
  std::vector<double> target_position_;
  const std::vector<double> joint_velocities_;
  const std::vector<std::string> joint_names_;
  double pub_rate_;

  // Thread stuff
  boost::thread worker_;
  boost::mutex mutex_;
  bool active_flag_;
};

void handleCommand(const sensor_msgs::JointStateConstPtr& states,
                   RobotSimulator& sim)
{
  sim.update(*states);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_simulator_node");
  ros::NodeHandle nh;

  // Load parameters
  double publish_rate;
  if (!nh.getParam("pub_rate", publish_rate)) {
    ROS_FATAL("Param 'pub_rate' must be set.");
    return -1;
  }

  std::vector<std::string> joint_names;
  if (!nh.getParam("controller_joint_names", joint_names)) {
    ROS_FATAL("Param 'controller_joint_names' must be set.");
    return -1;
  }

  RobotSimulator sim (publish_rate, joint_names);

  ros::Subscriber command_sub =
      nh.subscribe<sensor_msgs::JointState>("target", 1,
                                            boost::bind(handleCommand, _1, boost::ref(sim)));

  ROS_INFO("Starting teleop robot simulator.");
  sim.start();
  ros::spin();

  return 0;
}
