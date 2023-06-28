add_executable(test_bev_cam test_cam_publish.cpp)
target_link_libraries(test_bev_cam
    bev_lidar_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)