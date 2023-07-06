#ifndef CAM_BEV
#define CAM_BEV

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

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
        std::string img1_topic = "/cam_front/raw";
        std::string img2_topic = "/cam_front_left/raw";
        std::string pub_img_topic = "/BEV_images";
        cv::Mat BEV_image;

        ros::Publisher pub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image1;   // topic1 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image2;   // topic2 输入
        message_filters::Synchronizer<SyncPolicy>* sync_;

        std::vector<cv::Mat> six_cam_images;
    public:
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg_cam1, 
                            const sensor_msgs::ImageConstPtr& msg_cam2);
        
        void CamPublisher(ros::NodeHandle nh);
        void CamSubscriber(ros::NodeHandle nh);

        cv::Mat BirdEyeView(std::vector<cv::Mat> images);
};
} // namespace bevlidar

#endif /* CAM_BEV */
