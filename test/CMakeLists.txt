add_executable(test_bev_cam test_bev_cam.cpp)
target_link_libraries(test_bev_cam
    bev_lidar_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)

add_executable(test_read_json test_read_json.cpp)
target_link_libraries(test_read_json
    bev_lidar_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
   ${JSON_INC_PATH}
)
