#ifndef CAM_BEV
#define CAM_BEV

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>

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
