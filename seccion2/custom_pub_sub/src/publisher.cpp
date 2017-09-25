// ROS includes
#include "ros/ros.h"
#include "custom_pub_sub/Memory.h"
// Other includes needed to read free memory (RAM)
#include <string>
#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>

void get_memory(custom_pub_sub::Memory& mem);
unsigned int get_number(std::string line);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "memory_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<custom_pub_sub::Memory>("memory", 1000);
    ros::Rate loop_rate(1); // 1 Hz
    custom_pub_sub::Memory msg;
    unsigned int counter = 0;
    while (ros::ok())
    {
        msg.header.seq = counter;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = std::string("0");
        get_memory(msg);
        ROS_INFO("Sending memory message of time: %d.%d", msg.header.stamp.sec, msg.header.stamp.nsec);
        pub.publish(msg);        
        loop_rate.sleep();
        counter++;
    }

    return 0;

}

void get_memory(custom_pub_sub::Memory& mem)
{

    // Open the system file that holds the memory info.
    std::ifstream infile("/proc/meminfo");
    std::string line;
    // The first line contains the total memory value. Interested.
    std::getline(infile, line);
    mem.total.data = get_number(line) / 1024.0;
    // The second line contains the free memory value. Interested!.
    std::getline(infile, line);
    mem.free.data = get_number(line) / 1024.0;
    // The third line contains the available memory value. Interested!.
    std::getline(infile, line);
    mem.available.data = get_number(line) / 1024.0;
    // The fourth line contains the buffers memory. Interested!.
    std::getline(infile, line);
    mem.buffers = get_number(line); // We store them in kilobytes.

}

unsigned int get_number(std::string line)
{
    // Split the line and store in a vector.
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of("\t "), boost::token_compress_on);
    // The memory value must be on the second position.
    std::string mem_value = tokens[1];
    return std::stoul(mem_value); // From kilobytes to megabytes.

}














