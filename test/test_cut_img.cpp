#include "bev_lidar_cali/image_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    ImageProcess processor;
    cv::Mat img = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img_front.jpg");
    img = processor.CutImage(img);
    cv::imshow("cut_img", img);
	cv::waitKey(0);
    return 0;
}