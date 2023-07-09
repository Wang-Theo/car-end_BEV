#ifndef PARAM_PROCESS
#define PARAM_PROCESS

#include <opencv2/opencv.hpp>

#include "json/json.h"

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <thread>

namespace bevlidar {
class ParamProcess{
    public:
        std::vector<cv::Point2f> ReadJsonTest(std::vector<cv::Point2f> real_points, std::string camera_name);
};
} // namespace bevlidar

#endif /* PARAM_PROCESS */
