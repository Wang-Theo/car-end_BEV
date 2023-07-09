cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_STANDARD 11)
project(bev_lidar_cali)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package( OpenCV REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

# add library
add_library(bev_lidar_cali
    src/camera_node.cpp
    src/param_process.cpp
    src/image_process.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JSON_INC_PATH}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} jsoncpp_lib)

add_subdirectory(test)


