#include "bev_lidar_cali/param_process.hpp"

namespace bevlidar {

void ParamProcess::ReadJsonTest(std::string camera_name){
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
}

std::vector<cv::Point3f> ParamProcess::GetPoints(std::vector<cv::Point3f> real_points, std::string camera_name){
    std::vector<cv::Point3f> img_points;

    Eigen::Matrix<double, 3, 1> translation_matrix;
    translation_matrix << 1.5508477543, 
                        -0.493404796419, 
                         1.49574800619;

    Eigen::Quaterniond rot(0.2060347966337182, -0.2026940577919598, 0.6824507824531167, -0.6713610884174485);
    Eigen::Matrix3d rotation_martix = rot.toRotationMatrix(); 

    Eigen::Matrix3d intrinsic_matirx;
    intrinsic_matirx << 1260.8474446004698, 0.0, 807.968244525554, 
                        0.0, 1260.8474446004698, 495.3344268742088,
                        0.0, 0.0, 1.0;
    std::cout<< "-----------------------------------------" << std::endl;
    std::cout<< camera_name << "\n" << std::endl;
    std::cout<<"translation: "<< "\n" << translation_matrix <<"\n" <<  std::endl;
    std::cout<<"rotation: "<< "\n" << rotation_martix <<"\n" <<  std::endl;
    std::cout<<"camera_intrinsic: "<< "\n" << intrinsic_matirx << "\n"<<std::endl;

    for(int i = 0; i<real_points.size(); i++){
        Eigen::Matrix<double, 3, 1> piont_input;
        piont_input << real_points[i].x, real_points[i].y, real_points[i].z;

        Eigen::Matrix<double, 3, 1> piont_at_camera_coordinate;
        piont_at_camera_coordinate = rotation_martix * piont_input + translation_matrix;

        Eigen::Matrix<double, 3, 1> piont_at_image;
        piont_at_image = (1/real_points[i].z) * intrinsic_matirx * piont_at_camera_coordinate;

        cv::Point3f img_point;
        img_point.x = piont_at_image(0,0);
        img_point.y = piont_at_image(1,0);
        img_point.z = piont_at_image(2,0);
        img_points.push_back(img_point);
    }

    return img_points;
}

} // namespace bevlidar