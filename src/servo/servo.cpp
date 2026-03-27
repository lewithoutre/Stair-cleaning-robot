#include "servo.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <stdexcept>

ServoInterface::ServoInterface(const std::string& port_name, int baud_rate)
    : port_name_(port_name), baud_rate_(baud_rate), serial_fd(-1) {
    if (!openPort()) {
        std::cerr << "[ServoInterface] Failed to open port " << port_name_ << " on creation." << std::endl;
    }
}

ServoInterface::~ServoInterface() {
    closePort();
}

bool ServoInterface::openPort() {
    if (serial_fd != -1) {
        closePort();
    }
    
    // 打开串口，O_RDWR读写模式，O_NOCTTY不作为控制终端，O_NDELAY非阻塞
    serial_fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        return false;
    }

    // 设置串口参数 (对应 ESP32 中的 SERIAL_8N1, 115200 波特率)
    struct termios options;
    tcgetattr(serial_fd, &options);

    // 设置波特率
    speed_t speed;
    switch (baud_rate_) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break; // 默认使用115200
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 设置数据位 8，无校验 N，停止位 1
    options.c_cflag |= (CLOCAL | CREAD); // 允许接收字符，忽略调制解调器控制线
    options.c_cflag &= ~PARENB;          // 无校验 (No Parity)
    options.c_cflag &= ~CSTOPB;          // 1个停止位 (1 Stop bit)
    options.c_cflag &= ~CSIZE;           // 清除数据位掩码
    options.c_cflag |= CS8;              // 8个数据位 (8 Data bits)

    // 选择原始输入和输出模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    options.c_oflag &= ~OPOST;

    // 清空缓冲区，将配置立即生效
    tcflush(serial_fd, TCIFLUSH);
    tcsetattr(serial_fd, TCSANOW, &options);

    return true;
}

void ServoInterface::closePort() {
    if (serial_fd != -1) {
        close(serial_fd);
        serial_fd = -1;
    }
}

void ServoInterface::sendString(const std::string& str) {
    if (serial_fd == -1) {
        std::cerr << "[ServoInterface] Port not open, cannot send: " << str;
        return;
    }
    ssize_t bytes_written = write(serial_fd, str.c_str(), str.length());
    if (bytes_written < 0) {
        std::cerr << "[ServoInterface] Error writing to serial port." << std::endl;
    }
}

// 控制单舵机
void ServoInterface::setServoPosition(int id, int position, int time) {
    char cmd[128];
    // 指令格式: #000P0500T1000! 带回车换行
    snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!\n", id, position, time);
    sendString(std::string(cmd));
}

// 控制多舵机同步
void ServoInterface::setMultipleServos(const std::vector<ServoData>& servos) {
    if (servos.empty()) return;

    std::string cmd = "{";
    char buf[128];
    for (const auto& s : servos) {
        snprintf(buf, sizeof(buf), "#%03dP%04dT%04d!", s.id, s.position, s.time);
        cmd += buf;
    }
    cmd += "}\n";
    sendString(cmd);
}

void ServoInterface::flip(double duration_sec) {
    std::cout << "[ServoInterface] flip placeholder, moving servos..." << std::endl;
    
    // 假设 0 号和 1 号舵机是执行 flip 动作的关键舵机，这里做个演示调用
    setServoPosition(0, 2000, static_cast<int>(duration_sec * 1000));
    setServoPosition(1, 2000, static_cast<int>(duration_sec * 1000));
    
    std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));

    // 回到原始位置
    setServoPosition(0, 500, static_cast<int>(duration_sec * 1000));
    setServoPosition(1, 500, static_cast<int>(duration_sec * 1000));
    
    std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
}
