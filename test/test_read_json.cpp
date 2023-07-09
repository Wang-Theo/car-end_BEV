#include "bev_lidar_cali/param_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    std::vector<cv::Point2f> real_points;
    std::vector<cv::Point2f> img_points;
    cv::Point2f P1(500.f, 600.f), P2(700.f, 600.f), 
                P3(400.f, 900.f), P4(800.f, 900.f);

    real_points.push_back(P1);
    real_points.push_back(P2);
    real_points.push_back(P3);
    real_points.push_back(P4);

    ParamProcess processor;
    img_points = processor.ReadJsonTest(real_points, "CAM_FRONT");
    return 0;
}