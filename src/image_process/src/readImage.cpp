#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ORBextractor.h"

using namespace std;
using namespace cv;

int main(int argc,char** argv){
    std::cout << "I get here!" << std::endl;
    ros::init(argc, argv, "readImage");
    ros::NodeHandle n;

    cv::Mat img = cv::imread(argv[1],-1);
    
    if(img.empty()) return -1;

    std::vector<KeyPoint> keypoints;
    Ptr<FeatureDetector> detector = ORB::create();
    cv::Mat img_with_orb;
    detector->detect(img, keypoints);
    drawKeypoints(img, keypoints, img_with_orb, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    cv::namedWindow("raw picture",cv::WINDOW_AUTOSIZE);
    cv::namedWindow("picture with orb",cv::WINDOW_AUTOSIZE);
    cv::imshow("raw picture",img);
    cv::imshow("picture with orb", img_with_orb);
    cv::waitKey(0);
    cv::destroyAllWindows();
    

    // cv::namedWindow("My Camera", cv::WINDOW_AUTOSIZE);
    // cv::VideoCapture cap;
    // if(argc==1){
    //     cap.open(0);  // open the first camera
    // }
    // else{
    //     cap.open(argv[1]);
    // }
    // if(!cap.isOpened()){
    //     std::cerr << "Couldn't open capture." << std::endl;
    //     return -1;
    // }
    // ros::spin();
    return 0;
}