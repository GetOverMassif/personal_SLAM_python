#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

int main(int argc,char** argv){
    // std::cout << "I get here!" << std::endl;
    ros::init(argc, argv, "readFromCam");
    int index = 0;
    ros::NodeHandle n;
    
    cv::namedWindow("My Camera", cv::WINDOW_AUTOSIZE);
    cv::VideoCapture cap;
    if(argc==1){
        cap.open(0);  // open the first camera
    }
    else{
        index = (int)argv[1][0]-48;
        cap.open(index);
    }
    if(!cap.isOpened()){
        std::cerr << "Couldn't open capture." << std::endl;
        return -1;
    }
    cv::Mat frame;
    int keyvalue;
    while (true)
    {
        cap >> frame;
        if(frame.empty()) break;
        cv::imshow("My Camera", frame);
        keyvalue = cv::waitKey(50);
        if(keyvalue==27) break;
    }
    cv::destroyAllWindows();
    
    // ros::spin();
    return 0;
}