

#include "bev_lidar_cali/cam_BEV.hpp"

namespace bevlidar {

void CamBEV::ImageCallback(const sensor_msgs::ImageConstPtr& msg){

}

void CamBEV::CamPublisher(ros::NodeHandle nh){
    // cv::VideoCapture cap;
    // int api_id=cv::CAP_ANY;
    // cap.open(device_id+api_id);

    // image_transport::ImageTransport it(nh);
    // image_transport::Publisher pub = it.advertise(img_topic.c_str(), 1);
 
    // ros::Rate loop_rate(30);
    // while (nh.ok()) {
	//     cap.read(frame);
	//     if(!frame.empty()){
	//     	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	//     	pub.publish(msg);
	//     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

}

void CamBEV::CamSubscriber(ros::NodeHandle nh){
    image_transport::ImageTransport it(nh); 
    image_transport::Subscriber sub = it.subscribe(img_topic.c_str(), 1, &CamBEV::ImageCallback, this);  
    ros::spin();  
}

} // namespace bevlidar