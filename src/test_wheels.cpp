#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>

// 包含我们原本写好的底盘轮子控制类
#include "controller.h"

int main(int argc, char** argv) {
    // 必须要初始化 ROS2 节点，这样才能发消息给 ESP32 底座
    rclcpp::init(argc, argv);
    
    std::cout << "=========================================" << std::endl;
    std::cout << "       Wheel Control Test Tool           " << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Make sure Micro-ROS ESP32 base is connected." << std::endl;
    std::cout << "Controls: [w] forward  [s] backward       " << std::endl;
    std::cout << "          [a] turn L   [d] turn R         " << std::endl;
    std::cout << "          [x] stop     [q] quit           " << std::endl;
    std::cout << "Type a letter and press ENTER." << std::endl;

    try {
        RobotController controller;
        char cmd;
        
        while (rclcpp::ok()) {
            std::cout << "\nCmd: ";
            std::cin >> cmd;
            
            if (cmd == 'q') {
                std::cout << "Stopping wheel and exiting..." << std::endl;
                controller.stop();
                break;
            }
            
            // 根据你的按键，调用 set_velocity(线速度X, 线速度Y, 角速度Z)
            switch (cmd) {
                case 'w': 
                    controller.set_velocity(0.15, 0.0, 0.0); 
                    std::cout << "-> FORWARD (0.15 m/s)\n"; 
                    break;
                case 's': 
                    controller.set_velocity(-0.15, 0.0, 0.0); 
                    std::cout << "-> BACKWARD (-0.15 m/s)\n"; 
                    break;
                case 'a': 
                    controller.set_velocity(0.0, 0.0, 0.5); 
                    std::cout << "-> TURN LEFT (0.5 rad/s)\n"; 
                    break;
                case 'd': 
                    controller.set_velocity(0.0, 0.0, -0.5); 
                    std::cout << "-> TURN RIGHT (-0.5 rad/s)\n"; 
                    break;
                case 'x': 
                    controller.stop(); 
                    std::cout << "-> STOP (0.0 m/s)\n"; 
                    break;
                default: 
                    std::cout << "Unknown cmd. Use w/a/s/d/x/q.\n"; 
                    break;
            }
            
            // 稍等一会让系统把这个网络状态消息发出去
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}