#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;

    if (argc > 1) {
        port = argv[1];
    }

    std::cout << "尝试连接到 ESP32: " << port << " (波特率 " << baud_rate << ")" << std::endl;

    int serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd == -1) {
        std::cerr << "❌ 连接串口 " << port << " 失败! 错误: " << std::strerror(errno) << std::endl;
        std::cerr << "请检查 USB 是否插好，或使用 sudo chmod 666 " << port << " 赋权。\n";
        return 1;
    }

    termios options{};
    tcgetattr(serial_fd, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    tcsetattr(serial_fd, TCSANOW, &options);
    tcflush(serial_fd, TCIOFLUSH);

    std::cout << "✅ 连接成功！\n";

    bool running = true;

    // 开启一个子线程专门用于打印来自 ESP32 的串口输出
    std::thread reader_thread([&]() {
        char buf[256];
        std::string line_buffer;
        while (running) {
            ssize_t n = read(serial_fd, buf, sizeof(buf) - 1);
            if (n > 0) {
                buf[n] = '\0';
                line_buffer += buf;
                size_t pos;
                while ((pos = line_buffer.find('\n')) != std::string::npos) {
                    std::string line = line_buffer.substr(0, pos);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    std::cout << "\r  [ESP32] " << line << "\n";
                    std::cout << "请输入测试命令 (1/2/3/q): " << std::flush;
                    line_buffer.erase(0, pos + 1);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    auto send_command = [&](const std::string& cmd) {
        std::string payload = cmd + "\n";
        write(serial_fd, payload.c_str(), payload.length());
    };

    while (running) {
        std::cout << "\n==============================\n";
        std::cout << " 🧹 楼梯清扫机器人 ESP32 调试终端 🔄\n";
        std::cout << "==============================\n";
        std::cout << "  [1] 打开清扫电机 (CLEAN_ON)\n";
        std::cout << "  [2] 关闭清扫电机 (CLEAN_OFF)\n";
        std::cout << "  [3] 触发爬楼梯翻转 (CLIMB)\n";
        std::cout << "  [q] 退出测试终端\n";
        std::cout << "==============================\n";
        std::cout << "请输入测试命令 (1/2/3/q): " << std::flush;

        std::string input;
        if (!std::getline(std::cin, input)) break;
        
        if (input == "1") {
            std::cout << "➡️ 发送指令: CLEAN_ON\n";
            send_command("CLEAN_ON");
        } else if (input == "2") {
            std::cout << "➡️ 发送指令: CLEAN_OFF\n";
            send_command("CLEAN_OFF");
        } else if (input == "3") {
            std::cout << "➡️ 发送指令: CLIMB\n";
            std::cout << "⏳ 等待 ESP32 执行翻转动作，等待接收 'DONE' 信号...\n";
            send_command("CLIMB");
        } else if (input == "q" || input == "Q") {
            running = false;
        } else if (!input.empty()) {
            std::cout << "无效输入，请重新输入。\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running = false;
    if (reader_thread.joinable()) {
        reader_thread.join();
    }
    close(serial_fd);
    std::cout << "退出终端。\n";
    return 0;
}
