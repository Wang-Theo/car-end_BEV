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
        std::string img_topic_front = "/cam_front/raw";
        std::string img_topic_front_left = "/cam_front_left/raw";
        std::string img_topic_front_right = "/cam_front_right/raw";
        std::string img_topic_back = "/cam_back/raw";
        std::string img_topic_back_left = "/cam_back_left/raw";
        std::string img_topic_back_right = "/cam_back_right/raw";
        std::string pub_img_topic = "/BEV_images";
        cv::Mat BEV_image;

        ros::Publisher pub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,
                                                                sensor_msgs::Image,sensor_msgs::Image,
                                                                sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image_front;   // topic1 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image_front_left;   // topic2 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image_front_right;   // topic3 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image_back;   // topic4 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image_back_left;   // topic5 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image_back_right;   // topic6 输入
        message_filters::Synchronizer<SyncPolicy>* sync_;

        std::vector<cv::Mat> six_cam_images;
        int flag;

    public:
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg_img_front, 
                            const sensor_msgs::ImageConstPtr& msg_img_front_left,
                            const sensor_msgs::ImageConstPtr& msg_img_front_right,
                            const sensor_msgs::ImageConstPtr& msg_img_back,
                            const sensor_msgs::ImageConstPtr& msg_img_back_left,
                            const sensor_msgs::ImageConstPtr& msg_img_back_right);
        
        void CamPublisher(ros::NodeHandle nh);
        void CamSubscriber(ros::NodeHandle nh);

        cv::Mat BirdEyeView(std::vector<cv::Mat> images);
        cv::Mat JoinImageDirect(std::vector<cv::Mat> images);
        cv::Mat OrbDetect(std::vector<cv::Mat> images);
};
} // namespace bevlidar

#endif /* CAM_BEV */
