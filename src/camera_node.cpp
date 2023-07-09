#include "bev_lidar_cali/camera_node.hpp"

namespace bevlidar {

void CamBEV::ImageCallback(const sensor_msgs::ImageConstPtr& msg_img_front, 
                            const sensor_msgs::ImageConstPtr& msg_img_front_left,
                            const sensor_msgs::ImageConstPtr& msg_img_front_right,
                            const sensor_msgs::ImageConstPtr& msg_img_back,
                            const sensor_msgs::ImageConstPtr& msg_img_back_left,
                            const sensor_msgs::ImageConstPtr& msg_img_back_right){
    cv::Mat image_front_left = cv_bridge::toCvShare(msg_img_front_left, "bgr8")->image;
    six_cam_images.push_back(image_front_left);
    cv::Mat image_front = cv_bridge::toCvShare(msg_img_front, "bgr8")->image;
    six_cam_images.push_back(image_front);
    cv::Mat image_front_right = cv_bridge::toCvShare(msg_img_front_right, "bgr8")->image;
    six_cam_images.push_back(image_front_right);
    cv::Mat image_back_right = cv_bridge::toCvShare(msg_img_back_right, "bgr8")->image;
    six_cam_images.push_back(image_back_right);
    cv::Mat image_back = cv_bridge::toCvShare(msg_img_back, "bgr8")->image;
    six_cam_images.push_back(image_back);
    cv::Mat image_back_left= cv_bridge::toCvShare(msg_img_back_left, "bgr8")->image;
    six_cam_images.push_back(image_back_left);

    if(image_front_left.data==NULL||image_front.data==NULL
        ||image_front_right.data==NULL||image_back_right.data==NULL
        ||image_back.data==NULL||image_back_left.data==NULL)
    {
        std::cout<<"no image received"<<std::endl;
    }

    /*
    six_cam_images[image_front_left, image_front, image_front_right, image_back_right, image_back, image_back_left]
    */

    // =====process images from 6 cameras======

    switch (flag)
    {
    case 1:
        BEV_image = image_processor_->BirdEyeView(six_cam_images);
        break;
    case 2:
        BEV_image = image_processor_->OrbDetect(six_cam_images);
        break;
    case 3:
        BEV_image = image_processor_->JoinImageDirect(six_cam_images);
        break;
    default:
        break;
    } 

    // =====process images from 6 cameras======

    six_cam_images.clear();
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV_image).toImageMsg();
    pub.publish(out_msg);
}

void CamBEV::CamPublisher(ros::NodeHandle nh){
    // image_transport::ImageTransport it(nh);
    // image_transport::Publisher pub = it.advertise(pub_img_topic.c_str(), 1);

    // ros::Rate loop_rate(30);
    // while (nh.ok()) {
	//     if(!BEV_image.empty()){
	//     	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV_image).toImageMsg();
	//     	pub.publish(msg);
	//     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

void CamBEV::CamSubscriber(ros::NodeHandle nh){
    subscriber_image_front = new message_filters::Subscriber<sensor_msgs::Image>(nh, img_topic_front.c_str(), 1);
    subscriber_image_front_left = new message_filters::Subscriber<sensor_msgs::Image>(nh, img_topic_front_left.c_str(), 1);
    subscriber_image_front_right = new message_filters::Subscriber<sensor_msgs::Image>(nh, img_topic_front_right.c_str(), 1);
    subscriber_image_back = new message_filters::Subscriber<sensor_msgs::Image>(nh, img_topic_back.c_str(), 1);
    subscriber_image_back_left = new message_filters::Subscriber<sensor_msgs::Image>(nh, img_topic_back_left.c_str(), 1);
    subscriber_image_back_right = new message_filters::Subscriber<sensor_msgs::Image>(nh, img_topic_back_right.c_str(), 1);

    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *subscriber_image_front, *subscriber_image_front_left, 
                                                                            *subscriber_image_front_right, *subscriber_image_back, 
                                                                            *subscriber_image_back_left, *subscriber_image_back_right);
    sync_->registerCallback(boost::bind(&CamBEV::ImageCallback, this, _1, _2, _3, _4, _5, _6));

    pub = nh.advertise<sensor_msgs::Image>(pub_img_topic.c_str(), 1);

    ros::spin();  
}

} // namespace bevlidar