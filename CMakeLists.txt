cmake_minimum_required(VERSION 3.5)
project(stair_detector_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(stair_detector
  src/main.cpp
  src/camera/detector.cpp
  src/control/controller.cpp
  src/servo/servo.cpp
)

target_include_directories(stair_detector PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(stair_detector PRIVATE realsense2 ${OpenCV_LIBS})

if(USE_ROS2)
  target_link_libraries(stair_detector PRIVATE rclcpp::rclcpp geometry_msgs::geometry_msgs)
endif()
