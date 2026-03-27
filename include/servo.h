#pragma once

#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

class ServoInterface {
public:
    // 初始化时传入串口设备路径（树莓派GPIO串口通常为 /dev/serial0 或 /dev/ttyS0，波特率 115200
    ServoInterface(const std::string& port_name = "/dev/serial0", int baud_rate = 115200);
    ~ServoInterface();

    // 打开或重开串口
    bool openPort();
    // 关闭串口
    void closePort();

    // 控制单个舵机的位置 （等同 ESP32 中：#000P0500T1000!）
    // id: 舵机ID (0-255), position: 位置 (500-2500), time: 运行时间 (ms)
    void setServoPosition(int id, int position, int time);

    // 控制多个舵机同步转动 （等同 ESP32 中：{#000P0500T1000!#001P0500T1000!}）
    struct ServoData {
        int id;
        int position;
        int time;
    };
    void setMultipleServos(const std::vector<ServoData>& servos);

    // 默认自带一个动作测试 (原本的占位符函数)
    void flip(double duration_sec = 0.8);

private:
    int serial_fd; // Linux 文件描述符
    std::string port_name_;
    int baud_rate_;

    // 往 Linux 串口底层发送 ASCII 字符串
    void sendString(const std::string& str);
};
