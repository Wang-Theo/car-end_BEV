#ifndef CAM_BEV
#define CAM_BEV

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <thread>

namespace bevlidar {
class CamBEV{
    public:
        std::string img_topic="camera/image";

    public:
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
        
        void CamPublisher(ros::NodeHandle nh);
        void CamSubscriber(ros::NodeHandle nh);
};
} // namespace bevlidar

#endif /* CAM_BEV */
