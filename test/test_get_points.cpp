#include "bev_lidar_cali/param_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    std::vector<cv::Point3f> real_points;
    std::vector<cv::Point3f> img_points;
    cv::Point3f P1(-10.f, 0.f, 20.f), P2(10.f, 0.f, 20.f), 
                P3(-10.f, 0.f, 40.f), P4(10.f, 0.f, 40.f);

    real_points.push_back(P1);
    real_points.push_back(P2);
    real_points.push_back(P3);
    real_points.push_back(P4);

    ParamProcess processor;
    img_points = processor.GetPoints(real_points, "CAM_FRONT");
    std::cout<<"4 points at image: " << "\n" ;
    for(int i =0; i<img_points.size(); i++)
        std::cout << "p" << i+1 << std::fixed << std::setprecision(2)<< img_points[i] <<  std::endl;
    return 0;
}