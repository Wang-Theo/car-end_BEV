

#include "bev_lidar_cali/cam_BEV.hpp"

namespace bevlidar {

void CamBEV::ImageCallback(const sensor_msgs::ImageConstPtr& msg_cam1, 
                            const sensor_msgs::ImageConstPtr& msg_cam2){
    cv::Mat image1 = cv_bridge::toCvShare(msg_cam1, "bgr8")->image;
    cv::Mat image2 = cv_bridge::toCvShare(msg_cam2, "bgr8")->image;

    if(image1.data==NULL||image2.data==NULL)
    {
        std::cout<<"no image received"<<std::endl;
    }

    cv::rotate(image1, BEV_images, cv ::ROTATE_90_CLOCKWISE);
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV_images).toImageMsg();
    pub.publish(out_msg);
}

void CamBEV::CamPublisher(ros::NodeHandle nh){
    // image_transport::ImageTransport it(nh);
    // image_transport::Publisher pub = it.advertise(pub_img_topic.c_str(), 1);

    // ros::Rate loop_rate(30);
    // while (nh.ok()) {
	//     if(!BEV_images.empty()){
	//     	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV_images).toImageMsg();
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