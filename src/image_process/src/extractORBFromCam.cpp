#include "ros/ros.h"
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc,char** argv){
    std::cout << "I get here!" << std::endl;
    ros::init(argc, argv, "extractORBFromCam");
    ros::NodeHandle n;
    
    cv::namedWindow("My Camera", cv::WINDOW_AUTOSIZE);
    cv::VideoCapture cap;
    if(argc==1){
        cap.open(0);  // open the first camera
    }
    else{
        cap.open(argv[1]);
    }
    if(!cap.isOpened()){
        std::cerr << "Couldn't open capture." << std::endl;
        return -1;
    }
    cv::Mat frame;
    LJ_SLAM::ORBextractor orb_extractor; 
    for(;;){
        cap >> frame;
        // frame = cv::imread("/home/lj/Documents/personal-slam-ros/src/image_process/img/1.png");
        if(frame.empty()) break;
        orb_extractor.processImg(frame);
        cv::Mat frame_with_orb = orb_extractor.imgWithORB();
        cv::imshow("My Camera", frame_with_orb);
        if( cv::waitKey(50)>=0 ) break;
    }
    
    return 0;
}