#include "ros/ros.h"
#include "text_to_speech/SaySomething.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "say_something_test");

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<text_to_speech::SaySomething>("say_something");
	text_to_speech::SaySomething srv;
	std::string phrase;
	std::string voice;
	while(true)
	{
		std::cout << "Enter phrase to say (or INTRO to stop): ";
		std::getline(std::cin, phrase);
		if(phrase.length() == 0)
		{
			break;
		}
		std::cout << "Enter voice to speak (or INTRO to stop): ";
		std::getline(std::cin, voice);
		if(voice.length() == 0)
		{
			break;
		}
		srv.request.phrase = phrase;
		srv.request.voice = voice;
		if (client.call(srv))
		{
			ROS_INFO("Play duration: %lf", srv.response.data.toSec());
		}
		else
		{
			ROS_ERROR("Failed to call service say_something");
		}
	}
	ROS_INFO("Exiting by request of user. Goodbye!");
	return 0;
}
