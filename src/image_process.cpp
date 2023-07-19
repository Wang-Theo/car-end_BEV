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

cv::Mat ImageProcess::FuseImage(cv::Mat image1, cv::Mat image2){
    cv::Mat output_image(image1.cols, image1.rows, CV_8UC3, cv::Scalar(0, 0, 0));

    // for(int i =0; i<output_image.cols; i++){
    //     for(int j=0; j<output_image.rows; j++){

    //         if(   (image1.at<cv::Vec3b>(i, j)[0] == 0
    //               && image1.at<cv::Vec3b>(i, j)[1] == 0
    //               && image1.at<cv::Vec3b>(i, j)[2] == 0)
    //             &&(image2.at<cv::Vec3b>(i, j)[0] != 0
    //               && image2.at<cv::Vec3b>(i, j)[1] != 0
    //               && image2.at<cv::Vec3b>(i, j)[2] != 0)){
    //                 output_image.at<cv::Vec3b>(i, j)[0] = image2.at<cv::Vec3b>(i, j)[0];
    //                 output_image.at<cv::Vec3b>(i, j)[1] = image2.at<cv::Vec3b>(i, j)[1];
    //                 output_image.at<cv::Vec3b>(i, j)[2] = image2.at<cv::Vec3b>(i, j)[2];
    //             }
    //         else if((image2.at<cv::Vec3b>(i, j)[0] == 0
    //               && image2.at<cv::Vec3b>(i, j)[1] == 0
    //               && image2.at<cv::Vec3b>(i, j)[2] == 0)
    //             &&(image1.at<cv::Vec3b>(i, j)[0] != 0
    //               && image1.at<cv::Vec3b>(i, j)[1] != 0
    //               && image1.at<cv::Vec3b>(i, j)[2] != 0)){
    //                 output_image.at<cv::Vec3b>(i, j)[0] = image1.at<cv::Vec3b>(i, j)[0];
    //                 output_image.at<cv::Vec3b>(i, j)[1] = image1.at<cv::Vec3b>(i, j)[1];
    //                 output_image.at<cv::Vec3b>(i, j)[2] = image1.at<cv::Vec3b>(i, j)[2];
    //             }
    //         else{
    //                 output_image.at<cv::Vec3b>(i, j)[0] = (image1.at<cv::Vec3b>(i, j)[0] + image2.at<cv::Vec3b>(i, j)[0])/2;
    //                 output_image.at<cv::Vec3b>(i, j)[1] = (image1.at<cv::Vec3b>(i, j)[1] + image2.at<cv::Vec3b>(i, j)[1])/2;
    //                 output_image.at<cv::Vec3b>(i, j)[2] = (image1.at<cv::Vec3b>(i, j)[2] + image2.at<cv::Vec3b>(i, j)[2])/2;
    //         }
    //     }
    // }
    cv::addWeighted(image1, 0.5, image2, 0.5, 0, output_image);
    return output_image;
}

cv::Mat ImageProcess::CutImageMask(cv::Mat image){
    // cut image
    cv::Mat mask;
    cv::GaussianBlur(image, mask, cv::Size(5,5), 0);
    cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
    cv::threshold(mask, mask, 1, 255, cv::THRESH_BINARY);

    // Shi-Tomasi corners detection
    // cv::Mat mask_with_corner = mask.clone();
    // std::vector<cv::Point2f> corners;   // corners's postion
	// int maxcorners = 4;                 // maximal corners' number to be detected
	// double qualityLevel = 0.1;          // minimal eigenvalue
	// double minDistance = 600;	        // minimal distance between corners
	// int blockSize = 25;
	// double  k = 0.04;                   // weight coefficient
 
	// cv::goodFeaturesToTrack(mask_with_corner, corners, maxcorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);
 
	// std::cout << "Corners number: " << corners.size() << std::endl; // output info

	// for (unsigned i = 0; i < corners.size(); i++)
	// {
	// 	circle(mask_with_corner, corners[i], 10, cv::Scalar(0,255,255),4);  // draw corners
	// 	std::cout << "Corner location: " << corners[i] << std::endl;     // output corners' position
	// }

    // detect outline
    // std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // int index = 0;
    // for (; index >= 0; index = hierarchy[index][0])
	// {
    //     cv::drawContours(image, contours, index, (0,0,255), 2,8,hierarchy);
	// }

    return mask;
}

cv::Mat ImageProcess::RotateImage(cv::Mat image, int w, int h, double angle, double scale){
    // rotate images
    cv::Point2f center;
    center.x = w/2; center.y = h/2;
    int bound_w = (h * fabs(sin(angle * CV_PI / 180)) + w * fabs(cos(angle * CV_PI / 180)));
    int bound_h = (h * fabs(cos(angle * CV_PI / 180)) + w * fabs(sin(angle * CV_PI / 180)));                  
    cv::Mat R = cv::getRotationMatrix2D(center, angle, scale);

    R.at<double>(0, 2) += (bound_w - w) / 2;
    R.at<double>(1, 2) += (bound_h - h) / 2;

    cv::warpAffine(image, image, R, cv::Size(bound_h,bound_w+600));
    return image;
}

cv::Mat ImageProcess::JoinBEVImage(){
    cv::Mat result_image;

    // rotate images
    image_front_left_ = RotateImage(image_front_left_, image_front_left_.cols, image_front_left_.rows, 55, 1);
    image_front_right_ = RotateImage(image_front_right_, image_front_right_.cols, image_front_right_.rows, -55, 1);
    cv::rotate(image_back_, image_back_, cv::ROTATE_180);
    cv::rotate(image_back_right_, image_back_right_, cv::ROTATE_180);
    cv::rotate(image_back_left_, image_back_left_, cv::ROTATE_180);
    image_back_right_ = RotateImage(image_back_right_, image_back_right_.cols, image_back_right_.rows, 70, 1);
    image_back_left_ = RotateImage(image_back_left_, image_back_left_.cols, image_back_left_.rows, -70, 1);

    // get images' width and height again
    int w1 = image_front_left_.cols;    int h1 = image_front_left_.rows;
	int w2 = image_front_.cols;         int h2 = image_front_.rows;
    int w3 = image_front_right_.cols;   int h3 = image_front_right_.rows;
    int w4 = image_back_right_.cols;    int h4 = image_back_right_.rows;
	int w5 = image_back_.cols;          int h5 = image_back_.rows;
    int w6 = image_back_left_.cols;     int h6 = image_back_left_.rows;

    cv::Mat mask_front_left = CutImageMask(image_front_left_);
    cv::Mat mask_front = CutImageMask(image_front_);
    cv::Mat mask_front_right = CutImageMask(image_front_right_);
    cv::Mat mask_back_right = CutImageMask(image_back_right_);
    cv::Mat mask_back = CutImageMask(image_back_);
    cv::Mat mask_back_left = CutImageMask(image_back_left_);
    
    // save image to test 
    // cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_front.jpg",image_front_);
    // cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_front_left.jpg",image_front_left_);
    // cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_front_right.jpg",image_front_right_);
    // cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_back.jpg",image_back_);
    // cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_back_left.jpg",image_back_left_);
    // cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/test_img/test_img_back_right.jpg",image_back_right_);

    // joint images
	int width = 5000; int height = 4000;
    result_image = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
	// cv::Mat result_image_1 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
	// cv::Mat ROI_1 = result_image_1(cv::Rect(0,                    0, w1, h1));
    // cv::Mat result_image_2 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
	// cv::Mat ROI_2 = result_image_2(cv::Rect(805,                  685-620,  w2, h2));
    // cv::Mat result_image_3 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
    // cv::Mat ROI_3 = result_image_3(cv::Rect(805+1202-319,         0, w3, h3));
    // cv::Mat result_image_4 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
    // cv::Mat ROI_4 = result_image_4(cv::Rect(805+1202-319+770-507, 1339-414, w4, h4));
    // cv::Mat result_image_5 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
    // cv::Mat ROI_5 = result_image_5(cv::Rect(20+1002-581,          1303-413+1162-107, w5, h5));
    // cv::Mat result_image_6 = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
    // cv::Mat ROI_6 = result_image_6(cv::Rect(20,                   1303-413, w6, h6));

    //============polygon corner points=============
    int x_fl_1 = 667; int y_fl_1 = 1190; int x_fl_2 = 1045; int y_fl_2 = 661; 
    int x_f_1 = 471;  int y_f_1 = 477;   int x_f_2 = 1122;  int y_f_2 = 478;
    int x_fr_1 = 480; int y_fr_1 = 640;  int x_fr_2 = 857;  int y_fr_2 = 1178;
    int x_bl_1 = 617; int y_bl_1 = 538;  int x_bl_2 = 846;  int y_bl_2 = 1135;
    int x_b_1 = 475;  int y_b_1 = 131;   int x_b_2 = 1148;  int y_b_2 = 132;
    int x_br_1 = 407; int y_br_1 = 1140; int x_br_2 = 625;  int y_br_2 = 527;
    //============polygon corner points=============

    //==========image position - start xy===========
    int X_fl = 0;                       int Y_fl = 0;
    int X_f = x_fl_2 - x_f_1;           int Y_f = y_fl_2 - y_f_1;
    int X_fr = X_f + x_f_2 - x_fr_1;    int Y_fr = 0;
    int X_bl = x_fl_1 - x_bl_1;         int Y_bl = y_fl_1 - y_bl_1;
    int X_b = X_bl + x_bl_2 - x_b_1;    int Y_b = Y_bl + y_bl_2 - y_b_1;
    int X_br = X_fr + x_fr_2 - x_br_2;  int Y_br = y_fr_2 - y_br_2;
    int vehicle_X = X_f+x_f_1;          int vehicle_Y = y_fl_1; 
    int vehicle_W = x_f_2 - x_f_1;      int vehicle_H = Y_b + y_b_1 - y_f_1;
    //==========image position - start xy===========

    // ratio_image_back = (1.0*(X_br + x_br_1 - x_bl_2))/(1.0*(x_b_2 - x_b_1));
    // std::cout << "\n"<<ratio_image_back << std::endl;

	cv::Mat ROI_1 = result_image(cv::Rect(X_fl, Y_fl, w1, h1));        // front_left
	cv::Mat ROI_2 = result_image(cv::Rect(X_f,  Y_f,  w2, h2));        // front
    cv::Mat ROI_3 = result_image(cv::Rect(X_fr, Y_fr, w3, h3));        // front_right
    cv::Mat ROI_4 = result_image(cv::Rect(X_br+20, Y_br, w4, h4));     // back_right
    cv::Mat ROI_5 = result_image(cv::Rect(X_b,  Y_b, w5, h5));         // back
    cv::Mat ROI_6 = result_image(cv::Rect(X_bl-2, Y_bl, w6, h6));      // back_left
    cv::Mat ROI_7 = result_image(cv::Rect(vehicle_X, vehicle_Y, vehicle_W, vehicle_H));    // vehicle_model

    //=============test1==================
	// cv::Mat ROI_1 = result_image(cv::Rect(0,                    0, w1, h1));
	// cv::Mat ROI_2 = result_image(cv::Rect(805,                  685-620,  w2, h2));
    // cv::Mat ROI_3 = result_image(cv::Rect(805+1202-319,         0, w3, h3));
    // cv::Mat ROI_4 = result_image(cv::Rect(805+1202-319+770-507, 1339-414, w4, h4));
    // cv::Mat ROI_5 = result_image(cv::Rect(20+1002-581,          1303-413+1162-107, w5, h5));
    // cv::Mat ROI_6 = result_image(cv::Rect(20,                   1303-413, w6, h6));
    // cv::Mat ROI_7 = result_image(cv::Rect(1205,                 685, 802, 1432-65));
    //=============test1==================

    image_front_left_.copyTo(ROI_1, mask_front_left);
    image_front_right_.copyTo(ROI_3, mask_front_right);
    image_front_.copyTo(ROI_2, mask_front);
	image_back_right_.copyTo(ROI_4, mask_back_right);
    image_back_left_.copyTo(ROI_6, mask_back_left);
    image_back_.copyTo(ROI_5, mask_back);

    // result_image = FuseImage(FuseImage(FuseImage(FuseImage(FuseImage(result_image_1, result_image_2), result_image_3), result_image_4), result_image_5), result_image_6);

    // add vehicle picture in middle
    cv::Mat vehicle_picture = cv::imread("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/vehicle_picture.jpeg");
    cv::resize(vehicle_picture, vehicle_picture, cv::Size(802,1432-65));
    vehicle_picture.copyTo(ROI_7);

    cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/result_image.jpg",result_image);
    if(result_image.empty()) std::cout<<"\n======fail to joint image======"<<std::endl;
    return result_image;
}

cv::Mat ImageProcess::PerspectiveTransform(cv::Mat image, std::vector<cv::Point3f> points){
    std::vector<cv::Point2f> corners;// 4 points in original image
    std::vector<cv::Point2f> corners_trans;// 4 points in transformed image

    //**parameters of ROI area cut from camera images**//
    float roi_x0=0;
    float roi_y0=0;
    float ROI_HEIGHT=5000;
    float ROI_WIDTH=1600;
    //************************//

    //=============test1==================
    // cv::Point2f P1(700.f, 600.f), P2(900.f, 600.f), 
    //             P3(600.f, 900.f), P4(1000.f, 900.f);
    //=============test1==================

    cv::Point2f P1, P2, P3, P4;
    P3.x = points[0].x; P3.y = 900 - points[0].y;
    P4.x = points[1].x; P4.y = 900 - points[1].y;
    P1.x = points[2].x; P1.y = 900 - points[2].y;
    P2.x = points[3].x; P2.y = 900 - points[3].y;

    corners.push_back(P1);
    corners.push_back(P2);
    corners.push_back(P3);
    corners.push_back(P4);

    // set width of perspective image
    float IPM_WIDTH=1600;
    float N=8;

    // make widith of perspcetive image as N * width of vehicle front part
    float sacale=(IPM_WIDTH/N)/ROI_WIDTH;
    float IPM_HEIGHT=ROI_HEIGHT*sacale;

    // initialize 
    cv::Mat dst=cv::Mat::zeros(IPM_HEIGHT+100,IPM_WIDTH,image.type());

    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),0));  //P2
    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),0));  //P3
    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),IPM_HEIGHT));   //P1
    corners_trans.push_back(cv::Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),IPM_HEIGHT));   //P4

    // compute transformation martix
    // auto warpMatrix_src2ipm = cv::getPerspectiveTransform(corners, corners_trans);
    auto warpMatrix_src2ipm = cv::findHomography(corners, corners_trans, cv::RANSAC);
    cv::warpPerspective(image, dst, warpMatrix_src2ipm, dst.size());

    // // mark points
    // for(int i=0;i<4;i++)
    //     circle(dst,corners_trans[i],5,cv::Scalar(0,255,255),4);
    return dst;
}

cv::Mat ImageProcess::BirdEyeView(std::vector<cv::Mat> images){
	cv::Mat result_image;

    // directly choose points in camera coordinate
    std::vector<cv::Point3f> real_points;
    std::vector<cv::Point3f> points;

    //=============test1==================
    // cv::Point3f P1(-5.f, -10.f, 20.f), P2(5.f, -10.f, 20.f), 
    //             P3(-5.f, -10.f, 30.f), P4(5.f, -10.f, 30.f);
    //=============test1==================

    cv::Point3f P1(-5.f, -10.f, 15.f), P2(5.f, -10.f, 15.f), 
                P3(-5.f, -10.f, 60.f), P4(5.f, -10.f, 60.f);

    real_points.push_back(P1);
    real_points.push_back(P2);
    real_points.push_back(P3);
    real_points.push_back(P4);

    // transform images
    ParamProcess processor;
    
    points = processor.GetPoints(real_points, "CAM_FRONT_LEFT");
    image_front_left_ = PerspectiveTransform(images[0], points);

    points = processor.GetPoints(real_points, "CAM_FRONT");
    image_front_ = PerspectiveTransform(images[1], points);
  
    points = processor.GetPoints(real_points, "CAM_FRONT_RIGHT");
    image_front_right_ = PerspectiveTransform(images[2], points);

    points = processor.GetPoints(real_points, "CAM_BACK_RIGHT");
    image_back_right_ = PerspectiveTransform(images[3], points);

    points = processor.GetPoints(real_points, "CAM_BACK");
    image_back_ = PerspectiveTransform(images[4], points);

    points = processor.GetPoints(real_points, "CAM_BACK_LEFT");
    image_back_left_ = PerspectiveTransform(images[5], points);

    //=============test1==================
    // cv::resize(image_back_,image_back_,cv::Size(image_back_.cols * 1.4557, image_back_.rows));
    //=============test1==================

    cv::resize(image_back_,image_back_,cv::Size(image_back_.cols * 1.49926, image_back_.rows));

    // join images
    result_image = JoinBEVImage();

    cv::imwrite("/home/renjie/workspace/catkin_ws/src/BEV_lidar_cali/images/result_image.jpg",result_image);
    if(result_image.empty()) std::cout<<"\n======fail to joint image======"<<std::endl;
    return result_image;
}

} // namespace bevlidar