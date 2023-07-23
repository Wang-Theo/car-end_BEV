#include "bev_lidar_cali/image_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    cv::Mat img_back_left = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_back_left.png");
    cv::Mat img_front_left = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_front_left.png");
    std::vector<cv::Mat> result;

    std::vector<cv::Mat> images;
    images.push_back(img_back_left);
    images.push_back(img_front_left);

    // stitcher feature detecting & matching
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

    // 1. detect feature points
    ImageProcess processor1;
    std::vector<cv::Mat> descriptors;
    cv::Mat result1 = processor1.OrbDetect(img_back_left, "CAM_BACK_LEFT");
    descriptors.push_back(processor1.descriptor);
    std::vector<cv::KeyPoint> kp1 = processor1.keypoint_filtered;

    ImageProcess processor2;
    cv::Mat result2 = processor2.OrbDetect(img_front_left, "CAM_FRONT_LEFT");
    descriptors.push_back(processor2.descriptor);
    std::vector<cv::KeyPoint> kp2 = processor2.keypoint_filtered;

    // draw and show
    int y1 = result1.cols; int x1 = result1.rows;
    int y2 = result2.cols; int x2 = result2.rows;
    cv::resize (result1, result1, cv::Size((y1 / 2), (x1 / 2)));
    cv::resize (result2, result2, cv::Size((y2 / 2), (x1 / 2)));
    cv::imshow("1", result1);
    cv::imshow("2", result2);
    cv::waitKey(0);

    // 2. match
    cv::Ptr<cv::BFMatcher> bfmatcher = cv::BFMatcher::create(cv::NORM_L2, true);
    std::vector<cv::DMatch> matches;
    bfmatcher->match(descriptors[0], descriptors[1], matches);

    // draw and show
    cv::Mat img_matches;
    drawMatches(img_back_left, kp1, img_front_left, kp2, matches, img_matches);
    imshow("BFMatcher", img_matches);
    cv::waitKey(0);

    return 0;
}