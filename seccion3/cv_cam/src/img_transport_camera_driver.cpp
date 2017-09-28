#include "opencv2/opencv.hpp"
#include <iostream>
#include <string> 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_driver");
    int width, height, fps;
    ros::NodeHandle nh;
    // Recover private parameters from the launch file
    ros::param::param<int>("~width", width, 640); // We still need the '~' character.
    ros::param::param<int>("~height", height, 360);
    ros::param::param<int>("~fps", fps, 20);
    // Open the default camera. 0 stands for /dev/video0
    cv::VideoCapture cap;    
    if(!cap.open(0))
    {
        std::cout << "Unable to open default camera device." << std::endl;
        return 0;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    // ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 20);
    // New Publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("image", 20);
    ros::Rate r(fps); 
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

