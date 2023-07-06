

#include "bev_lidar_cali/cam_BEV.hpp"

namespace bevlidar {

cv::Mat CamBEV::BirdEyeView(std::vector<cv::Mat> images){
	cv::Mat result_image;

    // joint images
    // to do......

    if(result_image.empty()) std::cout<<"\n======fail to joint image======"<<std::endl;
    return images[0];
}

void CamBEV::ImageCallback(const sensor_msgs::ImageConstPtr& msg_cam1, 
                            const sensor_msgs::ImageConstPtr& msg_cam2){
    cv::Mat image1 = cv_bridge::toCvShare(msg_cam1, "bgr8")->image;
    six_cam_images.push_back(image1);
    cv::Mat image2 = cv_bridge::toCvShare(msg_cam2, "bgr8")->image;
    six_cam_images.push_back(image2);

    if(image1.data==NULL||image2.data==NULL)
    {
        std::cout<<"no image received"<<std::endl;
    }

    BEV_image = BirdEyeView(six_cam_images);
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
    subscriber_image1 = new message_filters::Subscriber<sensor_msgs::Image>(nh, img1_topic.c_str(), 1);
    subscriber_image2 = new message_filters::Subscriber<sensor_msgs::Image>(nh, img2_topic.c_str(), 1);

    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *subscriber_image1, *subscriber_image2);
    sync_->registerCallback(boost::bind(&CamBEV::ImageCallback, this, _1, _2));

    pub = nh.advertise<sensor_msgs::Image>(pub_img_topic.c_str(), 1);

    ros::spin();  
}

} // namespace bevlidar