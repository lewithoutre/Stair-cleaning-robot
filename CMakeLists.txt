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
  src/climb/climb_action.cpp
  src/servo/servo.cpp
)

target_include_directories(stair_detector PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(stair_detector PRIVATE realsense2 ${OpenCV_LIBS} rclcpp::rclcpp)

if(TARGET geometry_msgs::geometry_msgs)
  target_link_libraries(stair_detector PRIVATE geometry_msgs::geometry_msgs)
endif()
