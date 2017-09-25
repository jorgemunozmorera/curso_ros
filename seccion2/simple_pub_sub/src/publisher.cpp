// ROS includes
#include "ros/ros.h"
#include "std_msgs/Float64.h"
// Other includes needed to read free memory (RAM)
#include <string>
#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>

void get_free_memory(double& mem);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "memory_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("memory", 1000);
    ros::Rate loop_rate(1); // 1 Hz
    std_msgs::Float64 msg;
    msg.data = 0.0;

    while (ros::ok())
    {
        get_free_memory(msg.data);
        ROS_INFO("Sending free memory value: %lf", msg.data);
        pub.publish(msg);        
        loop_rate.sleep();
    }

    return 0;
}

void get_free_memory(double& mem)
{

    // Open the system file that holds the memory info.
    std::ifstream infile("/proc/meminfo");
    std::string line;
    // The first line contains the total memory value. Not interested.
    std::getline(infile, line);
    // The second line contains the free memory value. Interested!.
    std::getline(infile, line);
    // Split the line and store in a vector.
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of("\t "),boost::token_compress_on);
    // The memory value must be on the second position.
    std::string mem_value = tokens[1];
    mem = std::stoul(mem_value) / 1024.0; // From kilobytes to megabytes.

}
