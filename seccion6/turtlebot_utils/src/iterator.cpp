#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Twist.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_iterator", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    rosbag::Bag bag;
    bag.open("/home/jorge/2017-09-20-12-13-11.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/turtle1/cmd_vel"));
    rosbag::View view(bag, rosbag::TopicQuery(topics)); // Create a view for the given topics (only one in our case)
    ros::Duration(2.0).sleep();
    ros::Rate r(0.5);
    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::Twist::ConstPtr msg = m.instantiate<geometry_msgs::Twist>();
        if (msg != NULL)
        {
            ROS_INFO("Topic: %s", m.getTopic().c_str());
            ROS_INFO("Time: %d.%d", m.getTime().sec, m.getTime().nsec);
            ROS_INFO("########");
    	    pub.publish(*msg); // msg is a pointer, so we use the * operator.
            r.sleep();
        }
    }
    bag.close();

    return 0;
}


