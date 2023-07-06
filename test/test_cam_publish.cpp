

#include "bev_lidar_cali/cam_BEV.hpp"

using namespace bevlidar;

void publish_cam(ros::NodeHandle nh, CamBEV cam_BEV){
    cam_BEV.CamPublisher(nh);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "img_publisher");
    ros::NodeHandle nh;
    CamBEV cam_BEV;
    cam_BEV.CamSubscriber(nh);
    return 0;
}