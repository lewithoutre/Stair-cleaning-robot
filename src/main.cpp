#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "climb_action.h"
#include "controller.h"
#include "detector.h"

namespace {

std::atomic_bool g_running {true};

enum class State {
    Search,
    Approach,
    Sweep,
    Climb,
    Recover,
    Finished,
    Error
};

struct AppConfig {
    int width = 640;
    int height = 480;
    bool visualize = false;

    double roi_height_ratio = 0.35;
    double roi_width_ratio = 0.40;
    double detect_dist = 0.75;
    double step_edge_thresh = 0.12;
    double climb_start_dist = 0.05; // ==== 已修改：设置为 0.05m (5厘米) 这意味着它需要极为贴近台阶才会停住起跳 ====

    double forward_speed = 0.10;
    double approach_speed = 0.06;
    double sweep_speed = 0.10;
    double sweep_sec = 2.5;
    int sweep_direction = 1;
    double post_climb_forward_sec = 1.0;
    double publish_period_sec = 0.05;

    int confirm_frames = 5;
    int lost_frames = 10;
    int max_stairs = 0;

    ClimbActionConfig climb;
};

void handle_signal(int) {
    g_running = false;
}

std::string env_string(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    return value ? std::string(value) : fallback;
}

int env_int(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    try {
        return std::stoi(value);
    } catch (...) {
        std::cerr << "[config] invalid int for " << name << ": " << value << std::endl;
        return fallback;
    }
}

double env_double(const char* name, double fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    try {
        return std::stod(value);
    } catch (...) {
        std::cerr << "[config] invalid double for " << name << ": " << value << std::endl;
        return fallback;
    }
}

bool env_bool(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    const std::string s = env_string(name, "");
    return s == "1" || s == "true" || s == "TRUE" || s == "yes" || s == "on";
}

bool has_display() {
    return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
}

AppConfig load_config() {
    AppConfig cfg;
    cfg.width = env_int("STAIR_CAMERA_WIDTH", cfg.width);
    cfg.height = env_int("STAIR_CAMERA_HEIGHT", cfg.height);
    cfg.visualize = env_bool("STAIR_VISUALIZE", has_display());

    cfg.roi_height_ratio = env_double("STAIR_ROI_HEIGHT", cfg.roi_height_ratio);
    cfg.roi_width_ratio = env_double("STAIR_ROI_WIDTH", cfg.roi_width_ratio);
    cfg.detect_dist = env_double("STAIR_DETECT_DIST", cfg.detect_dist);
    cfg.step_edge_thresh = env_double("STAIR_STEP_EDGE", cfg.step_edge_thresh);
    cfg.climb_start_dist = env_double("STAIR_CLIMB_START_DIST", cfg.climb_start_dist);

    cfg.forward_speed = env_double("STAIR_FORWARD_SPEED", cfg.forward_speed);
    cfg.approach_speed = env_double("STAIR_APPROACH_SPEED", cfg.approach_speed);
    cfg.sweep_speed = env_double("STAIR_SWEEP_SPEED", cfg.sweep_speed);
    cfg.sweep_sec = env_double("STAIR_SWEEP_SEC", cfg.sweep_sec);
    cfg.sweep_direction = env_int("STAIR_SWEEP_DIRECTION", cfg.sweep_direction) < 0 ? -1 : 1;
    cfg.post_climb_forward_sec = env_double("STAIR_POST_CLIMB_FORWARD_SEC", cfg.post_climb_forward_sec);
    cfg.publish_period_sec = env_double("STAIR_PUBLISH_PERIOD_SEC", cfg.publish_period_sec);

    cfg.confirm_frames = env_int("STAIR_CONFIRM_FRAMES", cfg.confirm_frames);
    cfg.lost_frames = env_int("STAIR_LOST_FRAMES", cfg.lost_frames);
    cfg.max_stairs = env_int("STAIR_MAX_STAIRS", cfg.max_stairs);

    cfg.climb.mode = ClimbAction::parse_mode(env_string("STAIR_CLIMB_MODE", "serial_wait"));
    cfg.climb.serial_port = env_string("STAIR_CLIMB_PORT", cfg.climb.serial_port);
    cfg.climb.serial_baud = env_int("STAIR_CLIMB_BAUD", cfg.climb.serial_baud);
    cfg.climb.start_command = env_string("STAIR_CLIMB_COMMAND", cfg.climb.start_command);
    cfg.climb.done_token = env_string("STAIR_CLIMB_DONE_TOKEN", cfg.climb.done_token);
    cfg.climb.timeout_sec = env_double("STAIR_CLIMB_TIMEOUT_SEC", cfg.climb.timeout_sec);
    cfg.climb.servo_port = env_string("STAIR_SERVO_PORT", cfg.climb.servo_port);
    cfg.climb.servo_baud = env_int("STAIR_SERVO_BAUD", cfg.climb.servo_baud);
    cfg.climb.servo_duration_sec = env_double("STAIR_SERVO_DURATION_SEC", cfg.climb.servo_duration_sec);

    if (cfg.publish_period_sec <= 0.0) cfg.publish_period_sec = 0.05;
    if (cfg.confirm_frames < 1) cfg.confirm_frames = 1;
    if (cfg.lost_frames < 1) cfg.lost_frames = 1;
    if (cfg.sweep_sec < 0.0) cfg.sweep_sec = 0.0;
    if (cfg.climb.timeout_sec < 0.0) cfg.climb.timeout_sec = 0.0;

    return cfg;
}

const char* state_name(State state) {
    switch (state) {
        case State::Search: return "SEARCH";
        case State::Approach: return "APPROACH";
        case State::Sweep: return "SWEEP";
        case State::Climb: return "CLIMB";
        case State::Recover: return "RECOVER";
        case State::Finished: return "FINISHED";
        case State::Error: return "ERROR";
    }
    return "UNKNOWN";
}

const char* climb_mode_name(ClimbActionMode mode) {
    switch (mode) {
        case ClimbActionMode::Disabled: return "disabled";
        case ClimbActionMode::Servo: return "servo";
        case ClimbActionMode::SerialTimed: return "serial_timed";
        case ClimbActionMode::SerialWaitDone: return "serial_wait_done";
    }
    return "unknown";
}

std::string fixed(double value, int precision = 2) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

bool metric_ready_to_climb(const DetectMetrics& m, const AppConfig& cfg) {
    if (m.valid_ratio <= 0.0) return false;
    const bool median_close = m.median_depth > 0.0 && m.median_depth <= cfg.climb_start_dist;
    const bool near_band_close = m.p10_depth > 0.0 && m.p10_depth <= cfg.climb_start_dist * 0.90;
    const bool many_close_pixels = m.close_ratio > 0.35;
    return median_close || near_band_close || many_close_pixels;
}

bool drive_for(RobotController& ctrl,
               double linear_x,
               double linear_y,
               double angular_z,
               double seconds,
               double publish_period_sec) {
    if (seconds <= 0.0) return g_running.load();

    const auto end_time = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(seconds);
    const auto period = std::chrono::duration<double>(publish_period_sec);

    while (g_running && std::chrono::steady_clock::now() < end_time) {
        ctrl.set_velocity(linear_x, linear_y, angular_z);
        std::this_thread::sleep_for(period);
    }
    ctrl.stop();
    return g_running.load();
}

bool run_sweep(RobotController& ctrl, const AppConfig& cfg) {
    const double y = cfg.sweep_speed * static_cast<double>(cfg.sweep_direction);

    std::cout << "[FSM] sweep left" << std::endl;
    if (!drive_for(ctrl, 0.0, y, 0.0, cfg.sweep_sec, cfg.publish_period_sec)) return false;

    std::cout << "[FSM] sweep right" << std::endl;
    if (!drive_for(ctrl, 0.0, -y, 0.0, cfg.sweep_sec * 2.0, cfg.publish_period_sec)) return false;

    std::cout << "[FSM] return to center" << std::endl;
    if (!drive_for(ctrl, 0.0, y, 0.0, cfg.sweep_sec, cfg.publish_period_sec)) return false;

    return true;
}

void draw_overlay(cv::Mat& vis,
                  const DetectMetrics& metrics,
                  State state,
                  bool detected,
                  int stair_count) {
    if (vis.empty()) return;
    if (metrics.roi_box.area() > 0) {
        cv::rectangle(vis, metrics.roi_box, detected ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 255), 2);
    }

    const std::string line1 = std::string("State ") + state_name(state) +
                              "  stairs " + std::to_string(stair_count);
    const std::string line2 = std::string("med ") + fixed(metrics.median_depth) +
                              "m p10 " + fixed(metrics.p10_depth) +
                              "m edge " + fixed(metrics.vertical_step);
    cv::putText(vis, line1, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);
    cv::putText(vis, line2, cv::Point(10, 58), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2);
}

void print_config_summary(const AppConfig& cfg) {
    std::cout << "[config] camera=" << cfg.width << "x" << cfg.height
              << " detect_dist=" << cfg.detect_dist
              << " climb_start=" << cfg.climb_start_dist
              << " forward=" << cfg.forward_speed
              << " approach=" << cfg.approach_speed
              << " max_stairs=" << cfg.max_stairs
              << " climb_mode=" << climb_mode_name(cfg.climb.mode)
              << " visualize=" << (cfg.visualize ? "on" : "off")
              << std::endl;
}

} // namespace

int main(int, char**) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const AppConfig cfg = load_config();
    print_config_summary(cfg);

    State state = State::Search;
    int confirmed_frames = 0;
    int lost_frames = 0;
    int stair_count = 0;
    auto last_log = std::chrono::steady_clock::now();

    try {
        RealSenseStairDetector detector(cfg.width, cfg.height, true);
        RobotController controller;
        ClimbAction climb_action(cfg.climb);

        while (g_running) {
            cv::Mat depth_u16;
            cv::Mat color_bgr;
            if (!detector.get_frames(depth_u16, color_bgr)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            auto [detected, metrics] = detector.detect_obstacle(
                depth_u16,
                cfg.roi_height_ratio,
                cfg.roi_width_ratio,
                cfg.detect_dist,
                cfg.step_edge_thresh
            );

            switch (state) {
                case State::Search:
                    controller.set_velocity(cfg.forward_speed, 0.0, 0.0);
                    confirmed_frames = detected ? confirmed_frames + 1 : 0;
                    if (confirmed_frames >= cfg.confirm_frames) {
                        controller.stop();
                        lost_frames = 0;
                        state = State::Approach;
                        std::cout << "[FSM] stair candidate confirmed, approaching slowly" << std::endl;
                    }
                    break;

                case State::Approach:
                    if (metrics.p10_depth < cfg.climb_start_dist) {
                        controller.stop();
                        state = State::Sweep;
                        std::cout << "[FSM] climb distance reached (dist=" << std::fixed << std::setprecision(2) << metrics.p10_depth << "m), start cleaning sweep" << std::endl;
                    } else {
                        controller.set_velocity(cfg.approach_speed, 0.0, 0.0);
                        lost_frames = detected ? 0 : lost_frames + 1;
                        if (lost_frames > cfg.lost_frames) {
                            controller.stop();
                            confirmed_frames = 0;
                            state = State::Search;
                            std::cout << "[FSM] candidate lost, resume searching" << std::endl;
                        }
                    }
                    break;

                case State::Sweep:
                    controller.stop();
                    if (!run_sweep(controller, cfg)) {
                        state = State::Error;
                        break;
                    }
                    state = State::Climb;
                    break;

                case State::Climb:
                    controller.stop();
                    if (!drive_for(controller, 0.0, 0.0, 0.0, 0.30, cfg.publish_period_sec)) {
                        state = State::Error;
                        break;
                    }
                    if (!climb_action.run_once(stair_count + 1)) {
                        state = State::Error;
                        g_running = false;
                        break;
                    }
                    ++stair_count;
                    state = State::Recover;
                    break;

                case State::Recover:
                    std::cout << "[FSM] recover forward after climb" << std::endl;
                    if (!drive_for(controller,
                                   cfg.forward_speed,
                                   0.0,
                                   0.0,
                                   cfg.post_climb_forward_sec,
                                   cfg.publish_period_sec)) {
                        state = State::Error;
                        break;
                    }
                    confirmed_frames = 0;
                    lost_frames = 0;
                    if (cfg.max_stairs > 0 && stair_count >= cfg.max_stairs) {
                        state = State::Finished;
                        g_running = false;
                    } else {
                        state = State::Search;
                    }
                    break;

                case State::Finished:
                case State::Error:
                    g_running = false;
                    break;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_log > std::chrono::seconds(1)) {
                last_log = now;
                std::cout << "[state] " << state_name(state)
                          << " detected=" << (detected ? "yes" : "no")
                          << " median=" << fixed(metrics.median_depth)
                          << " p10=" << fixed(metrics.p10_depth)
                          << " close=" << fixed(metrics.close_ratio, 2)
                          << " edge=" << fixed(metrics.vertical_step)
                          << std::endl;
            }

            if (cfg.visualize) {
                cv::Mat vis = detector.colorize_depth(depth_u16);
                draw_overlay(vis, metrics, state, detected, stair_count);
                cv::imshow("stair depth", vis);
                const int key = cv::waitKey(1);
                if (key == 'q' || key == 27) {
                    g_running = false;
                }
            }
        }

        controller.stop();
        detector.stop();
    } catch (const std::exception& e) {
        std::cerr << "[fatal] " << e.what() << std::endl;
        state = State::Error;
    }

    if (cfg.visualize) {
        cv::destroyAllWindows();
    }

    std::cout << "[exit] state=" << state_name(state)
              << " stairs=" << stair_count << std::endl;
    return state == State::Error ? 1 : 0;
}
