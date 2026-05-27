#include "climb_action.h"

#include "servo.h"

#include <sys/ioctl.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

speed_t baud_to_termios(int baud) {
    switch (baud) {
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

// 释放 DTR / RTS，避免 ESP32-S3 被串口打开动作拉进 DOWNLOAD 模式
bool release_dtr_rts(int fd) {
    int flags = TIOCM_DTR | TIOCM_RTS;

    // TIOCMBIC = bit clear，把 DTR/RTS 这两个 modem control 线清掉
    if (ioctl(fd, TIOCMBIC, &flags) != 0) {
        std::cerr << "[climb] warning: failed to release DTR/RTS: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

// 统一把命令结尾变成 \r\n，兼容 ESP-IDF stdin 的 CR/CRLF 行结束
std::string ensure_crlf_line_ending(std::string command) {
    while (!command.empty() && (command.back() == '\n' || command.back() == '\r')) {
        command.pop_back();
    }

    command += "\r\n";
    return command;
}

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

} // namespace

ClimbAction::ClimbAction(const ClimbActionConfig& config)
    : config_(config) {
    if (config_.mode == ClimbActionMode::Servo) {
        servo_ = std::make_unique<ServoInterface>(config_.servo_port, config_.servo_baud);
    }
}

ClimbAction::~ClimbAction() {
    close_serial();
}

bool ClimbAction::is_enabled() const {
    return config_.mode != ClimbActionMode::Disabled;
}

ClimbActionMode ClimbAction::parse_mode(const std::string& value) {
    const std::string mode = lower_copy(value);

    if (mode == "off" || mode == "disabled" || mode == "none") {
        return ClimbActionMode::Disabled;
    }

    if (mode == "servo" || mode == "servo_flip") {
        return ClimbActionMode::Servo;
    }

    if (mode == "serial_wait" || mode == "serial_ack" || mode == "serial_done") {
        return ClimbActionMode::SerialWaitDone;
    }

    return ClimbActionMode::SerialTimed;
}

bool ClimbAction::run_once(int stair_index) {
    if (config_.mode == ClimbActionMode::Disabled) {
        std::cout << "[ClimbAction] disabled, skip climb for stair "
                  << stair_index << std::endl;
        return true;
    }

    if (config_.mode == ClimbActionMode::Servo) {
        if (!servo_) {
            servo_ = std::make_unique<ServoInterface>(config_.servo_port, config_.servo_baud);
        }

        std::cout << "[ClimbAction] running servo fallback for stair "
                  << stair_index << std::endl;

        servo_->flip(config_.servo_duration_sec);
        return true;
    }

    if (!open_serial()) {
        std::cerr << "[ClimbAction] cannot open climb-master serial port "
                  << config_.serial_port << std::endl;
        return false;
    }

    std::cout << "[ClimbAction] send climb command for stair "
              << stair_index << " on " << config_.serial_port << std::endl;

    if (!send_serial_command(config_.start_command)) {
        return false;
    }

    if (config_.mode == ClimbActionMode::SerialWaitDone) {
        return wait_for_done_token();
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(config_.timeout_sec));
    return true;
}

bool ClimbAction::open_serial() {
    if (serial_fd_ != -1) {
        return true;
    }

    serial_fd_ = open(config_.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1) {
        std::cerr << "[ClimbAction] open failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    // 关键：打开串口后立刻释放 DTR/RTS
    // 这一步对应你 test_esp32_commands 里验证有效的做法
    release_dtr_rts(serial_fd_);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    termios options {};
    if (tcgetattr(serial_fd_, &options) != 0) {
        std::cerr << "[ClimbAction] tcgetattr failed: "
                  << std::strerror(errno) << std::endl;
        close_serial();
        return false;
    }

    cfmakeraw(&options);

    speed_t speed = baud_to_termios(config_.serial_baud);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);

    // 关键：关闭 hang-up-on-close，避免关闭串口时触发 ESP32 复位
    options.c_cflag &= ~HUPCL;

#ifdef CRTSCTS
    // 关闭硬件流控，避免 RTS 被当作流控线乱动
    options.c_cflag &= ~CRTSCTS;
#endif

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        std::cerr << "[ClimbAction] tcsetattr failed: "
                  << std::strerror(errno) << std::endl;
        close_serial();
        return false;
    }

    // 设置完 termios 后再释放一次，防止驱动或 tcsetattr 又改变了 DTR/RTS 状态
    release_dtr_rts(serial_fd_);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    tcflush(serial_fd_, TCIOFLUSH);

    return true;
}

void ClimbAction::close_serial() {
    if (serial_fd_ != -1) {
        // 关闭前再释放一次 DTR/RTS，尽量避免 ESP32 被拉进下载模式
        release_dtr_rts(serial_fd_);

        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool ClimbAction::send_serial_command(const std::string& command) {
    if (!open_serial()) {
        std::cerr << "[ClimbAction] cannot open serial port "
                  << config_.serial_port << std::endl;
        return false;
    }

    // 关键：统一使用 \r\n
    // 即使 main 里传的是 "CLEAN_ON\n"，这里也会先去掉旧结尾，再改成 "CLEAN_ON\r\n"
    std::string safe_command = ensure_crlf_line_ending(command);

    const char* data = safe_command.data();
    size_t remaining = safe_command.size();

    while (remaining > 0) {
        ssize_t written = write(serial_fd_, data, remaining);

        if (written < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            std::cerr << "[ClimbAction] write failed: "
                      << std::strerror(errno) << std::endl;
            return false;
        }

        data += written;
        remaining -= static_cast<size_t>(written);
    }

    tcdrain(serial_fd_);
    return true;
}

bool ClimbAction::wait_for_done_token() {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.timeout_sec);

    std::string buffer;
    char chunk[128];

    while (std::chrono::steady_clock::now() < deadline) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(serial_fd_, &read_set);

        timeval timeout {};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        int ret = select(serial_fd_ + 1, &read_set, nullptr, nullptr, &timeout);

        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "[ClimbAction] select failed: "
                      << std::strerror(errno) << std::endl;
            return false;
        }

        if (ret == 0) {
            continue;
        }

        ssize_t n = read(serial_fd_, chunk, sizeof(chunk));

        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }

            std::cerr << "[ClimbAction] read failed: "
                      << std::strerror(errno) << std::endl;
            return false;
        }

        if (n == 0) {
            continue;
        }

        buffer.append(chunk, chunk + n);

        // 可选：打印 ESP32 返回内容，方便调试
        std::cout.write(chunk, n);
        std::cout.flush();

        if (buffer.find(config_.done_token) != std::string::npos) {
            std::cout << "[ClimbAction] climb-master reported done" << std::endl;
            return true;
        }

        if (buffer.size() > 4096) {
            buffer.erase(0, buffer.size() - 1024);
        }
    }

    std::cerr << "[ClimbAction] timeout waiting for token: "
              << config_.done_token << std::endl;
    return false;
}