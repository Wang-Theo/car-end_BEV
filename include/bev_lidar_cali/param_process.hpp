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
        void ReadJsonTest(std::string camera_name);
        std::vector<cv::Point3f> GetPoints(std::vector<cv::Point3f> real_points, std::string camera_name);
};
} // namespace bevlidar

#endif /* PARAM_PROCESS */
