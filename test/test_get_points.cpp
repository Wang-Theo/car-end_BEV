#include "bev_lidar_cali/param_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    std::vector<cv::Point3f> real_points;
    std::vector<cv::Point3f> img_points;
    cv::Point3f P1(50.f, 60.f, 1.0f), P2(70.f, 60.f, 1.0f), 
                P3(40.f, 90.f, 1.0f), P4(80.f, 90.f, 1.0f);

    real_points.push_back(P1);
    real_points.push_back(P2);
    real_points.push_back(P3);
    real_points.push_back(P4);

    ParamProcess processor;
    img_points = processor.GetPoints(real_points, "CAM_FRONT");
    std::cout<<"4 points at image: "<< "\n" << img_points <<"\n" <<  std::endl;
    return 0;
}