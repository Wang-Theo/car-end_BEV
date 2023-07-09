#ifndef IAMGE_PROCESS
#define IAMGE_PROCESS

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>


#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <thread>

namespace bevlidar {
class ImageProcess{
    public:
        cv::Mat BirdEyeView(std::vector<cv::Mat> images);
        cv::Mat PerspectiveTransform(cv::Mat image);
        cv::Mat JoinImageDirect(std::vector<cv::Mat> images);
        cv::Mat OrbDetect(std::vector<cv::Mat> images);
};
} // namespace bevlidar

#endif /* IAMGE_PROCESS */
