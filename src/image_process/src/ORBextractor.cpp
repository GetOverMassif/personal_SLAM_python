#include "ORBextractor.h"
#include <opencv/cv.hpp>

namespace LJ_SLAM{

void ORBextractor::processImg(cv::Mat img_){
    img = img_;
    int col = img.cols;
    int row = img.rows;
    // cout << "[row,col]=[" << row << "," << col << "]" << endl;
    int iniThFAST = 20;
    FAST(img.rowRange(0,row).colRange(0,col),keypoints,iniThFAST,true);
}

cv::Mat ORBextractor::imgWithORB(){
    // img(cv::Rect(0,0,100,100)).setTo(0);
    // Ptr<FeatureDetector> detector = ORB::create();
    // detector->detect(img, keypoints);
    cv::Mat outimg1;
    drawKeypoints(img, keypoints, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    return outimg1;
}


} // namespace LJ_SLAM
