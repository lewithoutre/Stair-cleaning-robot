#pragma once

#include <chrono>
#include <iostream>
#include <string>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
#endif

class RobotController {
public:
    RobotController();
    ~RobotController();

    // set linear x (m/s), linear y (m/s) and angular z (rad/s)
    void set_velocity(double linear_x, double linear_y, double angular_z);
    void stop();

private:
#ifdef USE_ROS2
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    std::thread spin_thread_;
    bool spinning_ = false;
    // Optional micro-ROS agent process id (0 if not started)
    pid_t agent_pid_ = 0;
    std::string agent_dev_;
    std::string agent_baud_;

    // start/stop micro-ROS agent (controlled by env var MICRO_ROS_AUTOSTART="1")
    void start_micro_ros_agent_if_requested();
    void stop_micro_ros_agent_if_started();
#endif
};
