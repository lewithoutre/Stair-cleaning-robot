#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>

namespace {

speed_t baud_to_termios(int baud_rate) {
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return B115200;
    }
}

// 关键函数：释放 DTR / RTS，避免 ESP32-S3 被拉进 DOWNLOAD 模式
bool release_dtr_rts(int fd) {
    int flags = TIOCM_DTR | TIOCM_RTS;

    // TIOCMBIC = bit clear，把 DTR/RTS 这两个 modem control 线清掉
    if (ioctl(fd, TIOCMBIC, &flags) != 0) {
        std::cerr << "⚠️ 释放 DTR/RTS 失败: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool configure_serial(int fd, int baud_rate) {
    // 打开串口后，先释放一次 DTR/RTS
    release_dtr_rts(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    termios options {};
    if (tcgetattr(fd, &options) != 0) {
        std::cerr << "❌ tcgetattr 失败: " << std::strerror(errno) << std::endl;
        return false;
    }

    cfmakeraw(&options);

    speed_t speed = baud_to_termios(baud_rate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);

    // 关键：关闭 hang-up-on-close，避免关闭串口时再次触发复位
    options.c_cflag &= ~HUPCL;

#ifdef CRTSCTS
    // 关闭硬件流控，避免 RTS 被串口驱动当作流控线乱动
    options.c_cflag &= ~CRTSCTS;
#endif

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::cerr << "❌ tcsetattr 失败: " << std::strerror(errno) << std::endl;
        return false;
    }

    // 设置完 termios 后，再释放一次 DTR/RTS，防止 tcsetattr 过程里驱动又改了状态
    release_dtr_rts(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    tcflush(fd, TCIOFLUSH);
    return true;
}

bool write_all(int fd, const std::string& data) {
    const char* p = data.data();
    size_t remaining = data.size();

    while (remaining > 0) {
        ssize_t n = write(fd, p, remaining);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            std::cerr << "❌ 串口写入失败: " << std::strerror(errno) << std::endl;
            return false;
        }

        p += n;
        remaining -= static_cast<size_t>(n);
    }

    tcdrain(fd);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    // 推荐你建立 udev 后使用 /dev/esp32_climb
    // 也可以运行时传入：
    // ./test_esp32_commands /dev/serial/by-id/usb-1a86_USB_Single_Serial_5C4C165045-if00
    std::string port = "/dev/esp32_climb";
    int baud_rate = 115200;

    if (argc > 1) {
        port = argv[1];
    }

    std::cout << "尝试连接到 ESP32: " << port
              << " (波特率 " << baud_rate << ")" << std::endl;

    int serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd == -1) {
        std::cerr << "❌ 连接串口 " << port << " 失败! 错误: "
                  << std::strerror(errno) << std::endl;
        std::cerr << "请检查 USB 是否插好，或者当前用户是否在 dialout 组。\n";
        std::cerr << "也可以临时执行: sudo chmod 666 " << port << "\n";
        return 1;
    }

    if (!configure_serial(serial_fd, baud_rate)) {
        close(serial_fd);
        return 1;
    }

    std::cout << "✅ 连接成功！\n";
    std::cout << "提示：如果仍然看到 waiting for download，说明 ESP32 仍在下载模式，"
                 "需要检查 BOOT/GPIO0 是否被拉低。\n";

    std::atomic_bool running {true};

    std::thread reader_thread([&]() {
        char buf[256];
        std::string line_buffer;

        while (running.load()) {
            ssize_t n = read(serial_fd, buf, sizeof(buf) - 1);

            if (n > 0) {
                buf[n] = '\0';
                line_buffer += buf;

                size_t pos = 0;
                while ((pos = line_buffer.find('\n')) != std::string::npos) {
                    std::string line = line_buffer.substr(0, pos);
                    if (!line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }

                    if (!line.empty()) {
                        std::cout << "\r  [ESP32] " << line << "\n";
                        std::cout << "请输入测试命令 (1/2/3/q): " << std::flush;
                    }

                    line_buffer.erase(0, pos + 1);
                }
            } else if (n < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                    std::cerr << "\n❌ 串口读取失败: " << std::strerror(errno) << std::endl;
                    running = false;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    auto send_command = [&](const std::string& cmd) {
        std::string payload = cmd + "\r\n";
        return write_all(serial_fd, payload);
    };

    while (running.load()) {
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
        if (!std::getline(std::cin, input)) {
            break;
        }

        if (input == "1") {
            std::cout << "➡️ 发送指令: CLEAN_ON\n";
            send_command("CLEAN_ON");
        } else if (input == "2") {
            std::cout << "➡️ 发送指令: CLEAN_OFF\n";
            send_command("CLEAN_OFF");
        } else if (input == "3") {
            std::cout << "➡️ 发送指令: CLIMB\n";
            std::cout << "⏳ 等待 ESP32 执行翻转动作，等待接收 DONE 信号...\n";
            send_command("CLIMB");
        } else if (input == "q" || input == "Q") {
            running = false;
            break;
        } else if (!input.empty()) {
            std::cout << "无效输入，请重新输入。\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running = false;

    if (reader_thread.joinable()) {
        reader_thread.join();
    }

    // 关闭前再释放一次 DTR/RTS
    release_dtr_rts(serial_fd);
    close(serial_fd);

    std::cout << "退出终端。\n";
    return 0;
}