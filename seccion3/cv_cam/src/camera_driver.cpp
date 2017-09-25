#include "opencv2/opencv.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    cv::VideoCapture cap;
    // Open the default camera. 0 stands for /dev/video0
    if(!cap.open(0))
    {
        std::cout << "Unable to open default camera device." << std::endl;
        return 0;
    }   
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
    cap.set(cv::CAP_PROP_FPS, 20);
    cv::namedWindow("Webcam",1);
    while(cv::waitKey(50) != 27) // Waits 50 milliseconds, if user presses 'ESC' then exit.
    {
        cv::Mat frame;
        //cap >> frame;
        cap.read(frame);
        cv::imshow("Webcam", frame);
    }

    return 0;
}

