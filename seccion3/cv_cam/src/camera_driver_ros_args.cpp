#include "opencv2/opencv.hpp"
#include <iostream>
#include <string> 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_driver");
    ros::NodeHandle nh;    
    cv::VideoCapture cap;
    // Open the default camera. 0 stands for /dev/video0
    if(!cap.open(0))
    {
        std::cout << "Unable to open default camera device." << std::endl;
        return 0;
    }
    std::string width_arg(argv[1]);
    std::string height_arg(argv[2]);
    std::string fps_arg(argv[3]);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, std::stoi(width_arg));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, std::stoi(height_arg));
    cap.set(cv::CAP_PROP_FPS, std::stoi(fps_arg));
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 20);
    ros::Rate r(std::stoi(fps_arg)); 
    sensor_msgs::Image ros_img;
    while(ros::ok()) 
    {
        cv::Mat frame;
        cap.read(frame); // or cap >> frame;
        // Encapsulate the Mat frame within a CvImage object.
        cv_bridge::CvImage opencv_img(std_msgs::Header(), "bgr8", frame);
        // Convert to sensor_msgs::Image.
        opencv_img.toImageMsg(ros_img);
        // Publish as usual.
        image_pub.publish(ros_img);
        r.sleep();
    }

    return 0;
}

