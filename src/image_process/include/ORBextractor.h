#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <iostream>
#include <opencv/cv.hpp>

using namespace cv;
using namespace std;

namespace LJ_SLAM{

class ORBextractor
{
public:
    ORBextractor(){};
    void processImg(cv::Mat);
    cv::Mat imgWithORB();
private:
    cv::Mat img;
    vector<cv::KeyPoint> keypoints;
};

} // namespace LJ_SLAM

#endif