#include "ros/ros.h"
#include "custom_pub_sub/Memory.h"
#include <iostream>

void memoryCallback(const custom_pub_sub::Memory::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "memory_subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("memory", 1000, memoryCallback);
    ros::spin();

    return 0;
}

void memoryCallback(const custom_pub_sub::Memory::ConstPtr& msg)
{

    ROS_INFO("Received memory message of time: %d.%d", msg->header.stamp.sec, msg->header.stamp.nsec);
    std::cout << "Total memory: " << msg->total.data << std::endl;
    std::cout << "Available memory: " << msg->available.data << std::endl;
    std::cout << "Free memory: " << msg->free.data << std::endl;
    std::cout << "Buffers memory: " << msg->buffers << std::endl;
    std::cout << std::endl;
}

