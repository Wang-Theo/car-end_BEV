

#include "bev_lidar_cali/cam_BEV.hpp"

namespace bevlidar {

cv::Mat CamBEV::OrbDetect(std::vector<cv::Mat> images){
    // ORB Detection
    cv::Mat result_image;
	// create ORB detection
	cv::Ptr<cv::ORB> orb = cv::ORB::create();
	// create descriptor  
	std::vector<cv::KeyPoint> keypoint;
    std::vector<cv::KeyPoint> keypoint_filtered;
	cv::Mat descriptor;
	// detect and draw
	orb->detectAndCompute(images[1], cv::Mat(), keypoint, descriptor);
    
    for(int i =0; i<keypoint.size(); i++){
        if(keypoint[i].pt.y>300)
            keypoint_filtered.push_back(keypoint[i]);
    }
    drawKeypoints(images[1], keypoint_filtered, result_image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

    cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/result_image.jpg",result_image);
    if(result_image.empty()) std::cout<<"\n======fail to detect orb======"<<std::endl;
    return result_image;
}

cv::Mat CamBEV::JoinImageDirect(std::vector<cv::Mat> images){
    // joint images
    cv::Mat result_image;
    int w1 = images[0].cols; int h1 = images[0].rows;
	int w2 = images[1].cols; int h2 = images[1].rows;
    int w3 = images[2].cols; int h3 = images[2].rows;
	int width = w1 + w2 + w3; int height = std::max(h1, std::max(h2, h3));
	result_image = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
	cv::Mat ROI_1 = result_image(cv::Rect(0, 0, w1, h1));
	cv::Mat ROI_2 = result_image(cv::Rect(w1, 0, w2, h2));
    cv::Mat ROI_3 = result_image(cv::Rect(w1 + w2, 0, w3, h3));
	images[0].copyTo(ROI_1);
	images[1].copyTo(ROI_2);
    images[2].copyTo(ROI_3);

    cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/result_image.jpg",result_image);
    if(result_image.empty()) std::cout<<"\n======fail to joint image======"<<std::endl;
    return result_image;
}

cv::Mat CamBEV::BirdEyeView(std::vector<cv::Mat> images){
	cv::Mat result_image;

    // to BEV
    cv::Point2f corners[4];// 4 points in original image
    cv::Point2f corners_trans[4];// 4 points in transformed image

    //**parameters of ROI area cut from camera images**//
    float roi_x0=0;
    float roi_y0=0;
    float ROI_HEIGHT=1600;
    float ROI_WIDTH=800;
    //************************//
    cv::Point2d P1(400.f, 600.f), P2(800.f, 600.f), 
                P3(400.f, 900.f), P4(800.f, 900.f);

    corners[0] = P1;
    corners[1] = P2;
    corners[2] = P3;
    corners[3] = P4;

    // set width of perspective image
    float IPM_WIDTH=1600;
    float N=5;
    // make widith of perspcetive image as N * width of vehicle front part
    float sacale=(IPM_WIDTH/N)/ROI_WIDTH;
    float IPM_HEIGHT=ROI_HEIGHT*sacale;

    // initialize 
    cv::Mat dst=cv::Mat::zeros(IPM_HEIGHT+50,IPM_WIDTH,images[1].type());

    corners_trans[0] = cv::Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),0);  //P2
    corners_trans[1] = cv::Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),0);  //P3
    corners_trans[2] = cv::Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),IPM_HEIGHT);   //P1
    corners_trans[3] = cv::Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),IPM_HEIGHT);   //P4

    // compute transformation martix
    auto warpMatrix_src2ipm = cv::getPerspectiveTransform(corners, corners_trans);
    cv::warpPerspective(images[1], dst, warpMatrix_src2ipm, dst.size());

    // mark points
    for(int i=0;i<4;i++)
        circle(dst,corners_trans[i],5,cv::Scalar(0,255,255),4);

    result_image = dst;

    cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/result_image.jpg",result_image);
    if(result_image.empty()) std::cout<<"\n======fail to joint image======"<<std::endl;
    return result_image;
}

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
        BEV_image = BirdEyeView(six_cam_images);
        break;
    case 2:
        BEV_image = OrbDetect(six_cam_images);
        break;
    case 3:
        BEV_image = JoinImageDirect(six_cam_images);
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