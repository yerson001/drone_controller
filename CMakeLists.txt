cmake_minimum_required(VERSION 3.0.2)
project(MarDrone)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_executable(app
  src/app_node.cpp
  src/drone_controller.cpp
)

target_link_libraries(app
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
