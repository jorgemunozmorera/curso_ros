#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <map>
#include <string>

void recognizerCallback(const std_msgs::String& msg);
// Create a dictionary to map words to commands.
std::map<std::string,std::string> media;
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "media_command", ros::init_options::AnonymousName);
    ROS_INFO("Listening for media...");
    // Map words to commands.
    media[std::string("up")]=std::string("xdotool key Up");
    media[std::string("down")]=std::string("xdotool key Down");
    media[std::string("next")]=std::string("xdotool key ctrl+Tab");
    media[std::string("previous")]=std::string("xdotool key ctrl+shift+Tab");
    media[std::string("news")]=std::string("firefox http://www.elmundo.es &"); // Look at the &, it is necessary to avoid blocking the execution.
    media[std::string("music")]=std::string("firefox https://www.youtube.com/watch?v=VMnjF1O4eH0 &");
    media[std::string("shopping")]=std::string("firefox https://www.amazon.es &");
    ros::NodeHandle nh;
    // Subscribe to recognizer topic.
    ros::Subscriber sub = nh.subscribe("/recognizer/output", 1000, recognizerCallback);
    ros::spin();
}

void recognizerCallback(const std_msgs::String& msg)
{
    ROS_INFO("I heard: %s", msg.data.c_str());
    std::string word = msg.data;
    // Execute the command assigned to the word.
    std::system(media[word].c_str());
    // If Up or Down, execute multiple times to scroll more.
    if(word.compare(std::string("up")) == 0 || word.compare(std::string("down")) == 0)
    {
        // To scroll a little bit more.
        ros::Duration(0.1).sleep();
        std::system(media[word].c_str());
        ros::Duration(0.1).sleep();
        std::system(media[word].c_str());
        ros::Duration(0.1).sleep();
        std::system(media[word].c_str()); 
    }
}

