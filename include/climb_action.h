#pragma once

#include <memory>
#include <string>

class ServoInterface;

enum class ClimbActionMode {
    Disabled,
    Servo,
    SerialTimed,
    SerialWaitDone
};

struct ClimbActionConfig {
    ClimbActionMode mode = ClimbActionMode::SerialWaitDone;
    std::string serial_port = "/dev/ttyUSB1";
    int serial_baud = 115200;
    std::string start_command = "CLIMB\n";
    std::string done_token = "DONE";
    double timeout_sec = 24.0;
    std::string servo_port = "/dev/serial0";
    int servo_baud = 115200;
    double servo_duration_sec = 0.8;
};

class ClimbAction {
public:
    explicit ClimbAction(const ClimbActionConfig& config);
    ~ClimbAction();

    ClimbAction(const ClimbAction&) = delete;
    ClimbAction& operator=(const ClimbAction&) = delete;

    bool run_once(int stair_index);
    bool is_enabled() const;

    static ClimbActionMode parse_mode(const std::string& value);

private:
    bool open_serial();
    void close_serial();
    bool send_serial_command(const std::string& command);
    bool wait_for_done_token();

    ClimbActionConfig config_;
    int serial_fd_ = -1;
    std::unique_ptr<ServoInterface> servo_;
};
