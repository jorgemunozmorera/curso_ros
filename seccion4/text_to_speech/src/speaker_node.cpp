#include "ros/ros.h"
#include "text_to_speech/SaySomething.h"
#include <sound_play/sound_play.h>
#include <unistd.h>

sound_play::SoundClient *sc_ptr; // Don't use globals. Ask the teacher about this if you want more info.
bool say(text_to_speech::SaySomething::Request &req, text_to_speech::SaySomething::Response &res);
int main(int argc, char **argv)
{
	ros::init(argc, argv, "say_something_server");
	ros::NodeHandle nh;
    sound_play::SoundClient sc;
    sc_ptr = &sc;
    ros::ServiceServer service = nh.advertiseService("say_something", say);
	ROS_INFO("Ready to say something with some voice...");
	ros::spin();

	return 0;
}

bool say(text_to_speech::SaySomething::Request &req, text_to_speech::SaySomething::Response &res)
{
	ROS_INFO("Request for saying '%s' with voice '%s'", req.phrase.c_str(), req.voice.c_str());
    sc_ptr->stopAll();    
    ros::Duration(1).sleep();
	sc_ptr->say(req.phrase, req.voice);
	ros::Duration(1).sleep();
    res.data = ros::Duration(2);
	return true;
}


