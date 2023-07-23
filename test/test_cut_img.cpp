#include "bev_lidar_cali/image_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    ImageProcess processor;
    cv::Mat img = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_front_right.png");
    img = processor.CutImageMask(img);
    std::cout << "image width: " << img.cols << std::endl;
    std::cout << "image height: " << img.rows << std::endl;
    cv::resize(img,img,cv::Size(1600,1000));
    cv::imshow("cut_img", img);
	cv::waitKey(0);
    return 0;
}