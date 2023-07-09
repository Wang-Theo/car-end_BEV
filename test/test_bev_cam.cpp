#include "bev_lidar_cali/camera_node.hpp"
#include "bev_lidar_cali/image_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    ros::init(argc, argv, "img_publisher");
    ros::NodeHandle nh;
    auto image_processor = std::make_shared<ImageProcess>();
    auto cam_BEV = std::make_shared<CamBEV>(image_processor);
    nh.getParam("flag", cam_BEV->flag);
    cam_BEV->CamSubscriber(nh);
    return 0;
}