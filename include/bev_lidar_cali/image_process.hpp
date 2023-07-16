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

#include "bev_lidar_cali/param_process.hpp"

namespace bevlidar {
class ImageProcess{
    public:
        cv::Mat image_front_left_;
        cv::Mat image_front_;
        cv::Mat image_front_right_;
        cv::Mat image_back_right_;
        cv::Mat image_back_;
        cv::Mat image_back_left_;

    public:
        cv::Mat BirdEyeView(std::vector<cv::Mat> images);
        cv::Mat PerspectiveTransform(cv::Mat image, std::vector<cv::Point3f> points);
        cv::Mat JoinImageDirect(std::vector<cv::Mat> images);
        cv::Mat FuseImage(cv::Mat image1, cv::Mat image2);
        cv::Mat RotateImage(cv::Mat image, int w, int h, double angle, double scale);
        cv::Mat CutImageMask(cv::Mat image);
        cv::Mat JoinBEVImage();
        cv::Mat OrbDetect(std::vector<cv::Mat> images);
};
} // namespace bevlidar

#endif /* IAMGE_PROCESS */
