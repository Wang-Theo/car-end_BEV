#include "bev_lidar_cali/param_process.hpp"

namespace bevlidar {

std::vector<cv::Point2f> ParamProcess::ReadJsonTest(std::vector<cv::Point2f> real_points, std::string camera_name){
    std::vector<cv::Point2f> img_points;
    Json::Reader reader;
	Json::Value root;
    std::ifstream in("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/cali_param.json", std::ios::binary);
    if (!in.is_open())
	{
		std::cout << "Error opening file" <<std::endl;
	}

	if (reader.parse(in, root)){
        std::string translation = root[camera_name]["translation"].asString();
        std::string rotation = root[camera_name]["rotation"].asString();
        std::string camera_intrinsic = root[camera_name]["camera_intrinsic"].asString();
        std::cout<< "-----------------------------------------" << std::endl;
        std::cout<< camera_name << std::endl;
        std::cout<<"translation: "<< translation << std::endl;
        std::cout<<"rotation: "<< rotation << std::endl;
        std::cout<<"camera_intrinsic: "<< camera_intrinsic << "\n"<<std::endl;
    }
    img_points = real_points;
    return real_points;
}

} // namespace bevlidar