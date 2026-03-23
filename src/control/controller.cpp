#include "controller.h"

#include <memory>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

RobotController::RobotController() {
    // Optionally start micro-ROS agent if requested via environment
    start_micro_ros_agent_if_requested();

    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("stair_controller_cpp");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spinning_ = true;
    spin_thread_ = std::thread([this]() {
        try { exec_->spin(); } catch(...) {}
    });
}

RobotController::~RobotController() {
    // stop executor and join thread
    try {
        spinning_ = false;
        if (exec_) exec_->cancel();
    } catch(...) {}
    try {
        if (spin_thread_.joinable()) spin_thread_.join();
    } catch(...) {}
    try { rclcpp::shutdown(); } catch(...) {}

    // stop agent if we started it
    stop_micro_ros_agent_if_started();
}


void RobotController::start_micro_ros_agent_if_requested() {
    const char *auto_start = std::getenv("MICRO_ROS_AUTOSTART");
    if (!auto_start) return;
    if (std::strcmp(auto_start, "1") != 0) return;

    const char *dev = std::getenv("MICRO_ROS_DEV");
    const char *baud = std::getenv("MICRO_ROS_BAUD");
    if (!dev) dev = "/dev/ttyUSB0";
    if (!baud) baud = "921600";

    agent_dev_ = dev;
    agent_baud_ = baud;

    // fork and exec micro_ros_agent
    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "[RobotController] failed to fork micro_ros_agent" << std::endl;
        return;
    }
    if (pid == 0) {
        // child
        const char *argv[] = {"micro_ros_agent", "serial", "--dev", agent_dev_.c_str(), "-b", agent_baud_.c_str(), nullptr};
        execvp("micro_ros_agent", const_cast<char* const*>(argv));
        // if execvp returns, error
        std::cerr << "[RobotController] exec micro_ros_agent failed: " << std::strerror(errno) << std::endl;
        _exit(127);
    }
    // parent
    agent_pid_ = pid;
    std::cout << "[RobotController] started micro_ros_agent pid=" << agent_pid_ << " dev=" << agent_dev_ << " baud=" << agent_baud_ << std::endl;
    // small delay to allow agent to initialize
    sleep(1);
}

void RobotController::stop_micro_ros_agent_if_started() {
    if (agent_pid_ == 0) return;
    std::cout << "[RobotController] stopping micro_ros_agent pid=" << agent_pid_ << std::endl;
    kill(agent_pid_, SIGTERM);
    int status = 0;
    waitpid(agent_pid_, &status, 0);
    agent_pid_ = 0;
}

void RobotController::set_velocity(double linear_x, double linear_y, double angular_z) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z;
    pub_->publish(msg);
}

void RobotController::stop() { set_velocity(0.0, 0.0, 0.0); }