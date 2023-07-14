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

void ParamProcess::GetCaliParam(std::string camera_name){
    if(camera_name == "CAM_FRONT"){
        translation_matrix << 1.5508477543, 
                            -0.493404796419, 
                            1.49574800619;

        Eigen::Quaterniond rot(0.2060347966337182, -0.2026940577919598, 0.6824507824531167, -0.6713610884174485);
        rotation_martix = rot.toRotationMatrix(); 

        intrinsic_matirx << 1260.8474446004698, 0.0, 807.968244525554, 
                            0.0, 1260.8474446004698, 495.3344268742088,
                            0.0, 0.0, 1.0;
    } else if(camera_name == "CAM_FRONT_LEFT"){
        translation_matrix << 1.03569100218, 
                            0.484795032713, 
                            1.59097014818;

        Eigen::Quaterniond rot(0.6924185592174665, -0.7031619420114925, -0.11648342771943819, 0.11203317912370753);
        rotation_martix = rot.toRotationMatrix(); 

        intrinsic_matirx << 1256.7414812095406, 0.0, 792.1125740759628, 
                            0.0, 1256.7414812095406, 492.7757465151356,
                            0.0, 0.0, 1.0; 
    } else if(camera_name == "CAM_FRONT_RIGHT"){
        translation_matrix << 1.52387798135, 
                            0.494631336551, 
                            1.50932822144;

        Eigen::Quaterniond rot(0.6757265034669446, -0.6736266522251881, 0.21214015046209478, -0.21122827103904068);
        rotation_martix = rot.toRotationMatrix(); 

        intrinsic_matirx << 1272.5979470598488, 0.0, 826.6154927353808, 
                            0.0, 1272.5979470598488, 479.75165386361925,
                            0.0, 0.0, 1.0; 
    } else if(camera_name == "CAM_BACK"){
        translation_matrix << 0.0283260309358, 
                            0.00345136761476, 
                            1.57910346144;

        Eigen::Quaterniond rot(0.5037872666382278, -0.49740249788611096, -0.4941850223835201, 0.5045496097725578);
        rotation_martix = rot.toRotationMatrix(); 

        intrinsic_matirx << 809.2209905677063, 0.0, 829.2196003259838, 
                            0.0, 809.2209905677063, 481.77842384512485,
                            0.0, 0.0, 1.0; 
    } else if(camera_name == "CAM_BACK_LEFT"){
        translation_matrix << 1.0148780988, 
                            -0.480568219723, 
                            1.56239545128;

        Eigen::Quaterniond rot(0.12280980120078765, -0.132400842670559, -0.7004305821388234, 0.690496031265798);
        rotation_martix = rot.toRotationMatrix(); 

        intrinsic_matirx << 1259.5137405846733, 0.0, 807.2529053838625, 
                            0.0, 1259.5137405846733, 501.19579884916527,
                            0.0, 0.0, 1.0; 
    } else if(camera_name == "CAM_BACK_RIGHT"){
        translation_matrix << 1.70079118954, 
                            0.0159456324149, 
                            1.51095763913;

        Eigen::Quaterniond rot(0.4998015430569128, -0.5030316162024876, 0.4997798114386805, -0.49737083824542755);
        rotation_martix = rot.toRotationMatrix(); 

        intrinsic_matirx << 1266.417203046554, 0.0, 816.2670197447984, 
                            0.0, 1266.417203046554, 491.50706579294757,
                            0.0, 0.0, 1.0; 
    }


    std::cout<< "-----------------------------------------" << std::endl;
    std::cout<< camera_name << "\n" << std::endl;
    std::cout<<"translation: "<< "\n" << translation_matrix <<"\n" <<  std::endl;
    std::cout<<"rotation: "<< "\n" << rotation_martix <<"\n" <<  std::endl;
    std::cout<<"camera_intrinsic: "<< "\n" << intrinsic_matirx << "\n"<<std::endl;

}

std::vector<cv::Point3f> ParamProcess::GetPoints(std::vector<cv::Point3f> real_points, std::string camera_name){
    std::vector<cv::Point3f> img_points;

    GetCaliParam(camera_name);
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