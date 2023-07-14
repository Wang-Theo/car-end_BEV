#ifndef PARAM_PROCESS
#define PARAM_PROCESS

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include<Eigen/Geometry>

#include "json/json.h"

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <thread>
#include <iomanip>

namespace bevlidar {
class ParamProcess{
    public:
        Eigen::Matrix<double, 3, 1> translation_matrix;
        Eigen::Matrix3d rotation_martix;
        Eigen::Matrix3d intrinsic_matirx;

    public:
        void ReadJsonTest(std::string camera_name);
        void GetCaliParam(std::string camera_name);
        std::vector<cv::Point3f> GetPoints(std::vector<cv::Point3f> real_points, std::string camera_name);
};
} // namespace bevlidar

#endif /* PARAM_PROCESS */
