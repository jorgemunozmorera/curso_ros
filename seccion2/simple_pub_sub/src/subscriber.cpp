#include "ros/ros.h"
#include "std_msgs/Float64.h"

void memoryCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "memory_subscriber", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("memory", 1000, memoryCallback);
  // In that case, spin is needed to check if messages arrived, because
  // callbacks execute in the main thread. Without spin, callbacks will
  // not be called.
  ros::spin();

  return 0;
}

// This callback DOES NOT EXECUTE on its own thread.
void memoryCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Received free memory: %lf", msg->data);
}

