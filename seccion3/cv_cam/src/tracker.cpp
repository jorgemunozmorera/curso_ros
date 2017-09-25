#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include <ros/ros.h>
using namespace std;
using namespace cv;
int main( int argc, char** argv ){
    // Show help
    if(argc < 2)
    {
        cout << "Usage: tracker <algorithm name>" << endl;
        return 0;
    }
    // Init ROS
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    // Declares all required variables
    Rect2d roi;
    Mat frame;
    // Set input video
    std::string algorithm = argv[1];

    // Create a tracker object
    // Possible algorithms: BOOSTING, KCF, TLD or MEDIANFLOW (OpenCV 3.1)
    // Don't use KCF in opencv 3.1, it has a bug.
    Ptr<Tracker> tracker = Tracker::create(algorithm.c_str());  
    VideoCapture cap;
    if(!cap.open(0))
    {
        std::cout << "Unable to open default camera device." << std::endl;
        return 0;
    }   
    // Set default values for camera, the lower the better perfomance
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 10);
    // Get bounding box
    cap >> frame;
    roi=selectROI("tracker",frame);
    // Quit if ROI was not selected
    if(roi.width==0 || roi.height==0)
    {
        return 0;
    }
    // Initialize tracker with first frame and bounding box
    tracker->init(frame,roi);
    // Perform the tracking process
    cout << "Start the tracking process, press CTRL + C to quit." << endl;
    ros::Rate r(10); // 10 hz
    while(ros::ok()) // Run while CTRL+C is not pressed in the terminal
    {
        // Get frame from the video
        cap >> frame;
        // Stop the program if no more images
        if(frame.rows==0 || frame.cols==0)
        {
            break;
        }
        // Update the tracking result
        tracker->update(frame,roi);
        // Draw the tracked object
        rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
        // Show image with the tracked object
        imshow("tracker",frame);
        // Still need waitkey        
        waitKey(1);
        r.sleep();
    }
    return 0;
}

