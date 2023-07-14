#include "bev_lidar_cali/image_process.hpp"

namespace bevlidar {

cv::Mat ImageProcess::OrbDetect(std::vector<cv::Mat> images){
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

cv::Mat ImageProcess::JoinImageDirect(std::vector<cv::Mat> images){
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

cv::Mat ImageProcess::PerspectiveTransform(cv::Mat image){
    std::vector<cv::Point2f> corners;// 4 points in original image
    std::vector<cv::Point2f> corners_trans;// 4 points in transformed image

    //**parameters of ROI area cut from camera images**//
    float roi_x0=0;
    float roi_y0=0;
    float ROI_HEIGHT=8000;
    float ROI_WIDTH=1600;
    //************************//

    cv::Point2f P1(500.f, 600.f), P2(700.f, 600.f), 
                P3(400.f, 900.f), P4(800.f, 900.f);

    // cv::Point2f P1(848.79, 962.41), P2(929.84, 915.20), 
    //             P3(506.84, 1196.05), P4(566.79, 1149.64);

    // cv::Point2f P1(495.33, 177.54),
    //             P2(495.33, 492.76),
    //             P3(495.33, 1438.39),
    //             P4(495.33, 1123.18);

    corners.push_back(P1);
    corners.push_back(P2);
    corners.push_back(P3);
    corners.push_back(P4);

    // set width of perspective image
    float IPM_WIDTH=1600;
    float N=5;
    // make widith of perspcetive image as N * width of vehicle front part
    float sacale=(IPM_WIDTH/N)/ROI_WIDTH;
    float IPM_HEIGHT=ROI_HEIGHT*sacale;

    // initialize 
    cv::Mat dst=cv::Mat::zeros(IPM_HEIGHT,IPM_WIDTH,image.type());

    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),0));  //P2
    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),0));  //P3
    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),IPM_HEIGHT));   //P1
    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),IPM_HEIGHT));   //P4

    // compute transformation martix
    // auto warpMatrix_src2ipm = cv::getPerspectiveTransform(corners, corners_trans);
    auto warpMatrix_src2ipm = cv::findHomography(corners, corners_trans, cv::RANSAC);
    cv::warpPerspective(image, dst, warpMatrix_src2ipm, dst.size());

    // mark points
    for(int i=0;i<4;i++)
        circle(dst,corners_trans[i],5,cv::Scalar(0,255,255),4);
    return dst;
}

cv::Mat ImageProcess::BirdEyeView(std::vector<cv::Mat> images){
	cv::Mat result_image;

    cv::Mat image_front_left_ = PerspectiveTransform(images[0]);
    cv::Mat image_front_ = PerspectiveTransform(images[1]);
    cv::Mat image_front_right_ = PerspectiveTransform(images[2]);

    // joint images
    int w1 = image_front_left_.cols; int h1 = image_front_left_.rows;
	int w2 = image_front_.cols; int h2 = image_front_.rows;
    int w3 = image_front_right_.cols; int h3 = image_front_right_.rows;
	int width = w1 + w2 + w3; int height = std::max(h1, std::max(h2, h3));
	result_image = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
	cv::Mat ROI_1 = result_image(cv::Rect(0, 0, w1, h1));
	cv::Mat ROI_2 = result_image(cv::Rect(w1, 0, w2, h2));
    cv::Mat ROI_3 = result_image(cv::Rect(w1 + w2, 0, w3, h3));

	image_front_left_.copyTo(ROI_1);
	image_front_.copyTo(ROI_2);
    image_front_right_.copyTo(ROI_3);

    cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/result_image.jpg",result_image);
    if(result_image.empty()) std::cout<<"\n======fail to joint image======"<<std::endl;
    return result_image;
}

} // namespace bevlidar