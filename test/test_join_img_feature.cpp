#include "bev_lidar_cali/image_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    ImageProcess processor;
    cv::Mat img_back_left = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_back_left.png");
    cv::Mat img_front_left = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_front_left.png");
    std::vector<cv::Mat> result;

    std::vector<cv::Mat> images;
    images.push_back(img_back_left);
    images.push_back(img_front_left);

    // cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
    // auto status = stitcher->stitch(images, result);
    // if (status == 0)
    // {
    //     cv::namedWindow("1", 0);
    //     cv::imshow("1", result);
    //     cv::waitKey();
    // }else{
    //     std::cout << "\nFail to find match feature !!!" << std::endl;
    // }

    result = processor.OrbDetect(images);
    cv::imshow("1", result[0]);
    cv::imshow("2", result[1]);
    cv::waitKey(0);
    return 0;
}