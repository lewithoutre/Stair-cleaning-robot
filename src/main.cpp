#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>

#include "detector.h"
#include "controller.h"
#include "servo.h"

enum class State { FORWARD, SWEEP, FLIP };

int main(int argc, char **argv) {
    // parameters (tweak as needed)
    int width = 640, height = 480;
    double detect_dist = 0.6;
    double step_var = 0.08;
    double forward_speed = 0.12;
    double sweep_angular = 0.6;
    double sweep_time = 0.8;
    double post_flip_forward = 1.0;

    RealSenseStairDetector det(width, height, true);
    RobotController ctrl;
    ServoInterface servo;

    State state = State::FORWARD;

    try {
        while (true) {
            cv::Mat depth_u16, color_bgr;
            if (!det.get_frames(depth_u16, color_bgr)) {
                // small sleep to avoid busy loop
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            auto [is_obs, metrics] = det.detect_obstacle(depth_u16, 0.35, 0.4, detect_dist, step_var);

            if (state == State::FORWARD) {
                ctrl.set_velocity(forward_speed, 0.0, 0.0);
                if (is_obs) {
                    std::cout << "Obstacle detected -> start sweep\n";
                    ctrl.stop();
                    state = State::SWEEP;
                }
            } else if (state == State::SWEEP) {
                std::cout << "Sweeping left\n";
                ctrl.set_velocity(0.0, 0.0, sweep_angular);
                std::this_thread::sleep_for(std::chrono::duration<double>(sweep_time));
                std::cout << "Sweeping right\n";
                ctrl.set_velocity(0.0, 0.0, -sweep_angular);
                std::this_thread::sleep_for(std::chrono::duration<double>(sweep_time));
                std::cout << "Centering and stop\n";
                ctrl.stop();
                state = State::FLIP;
            } else if (state == State::FLIP) {
                std::cout << "Calling servo flip\n";
                servo.flip();
                ctrl.set_velocity(forward_speed, 0.0, 0.0);
                std::this_thread::sleep_for(std::chrono::duration<double>(post_flip_forward));
                ctrl.stop();
                state = State::FORWARD;
            }

            // visualization
            cv::Mat vis = det.colorize_depth(depth_u16);
            cv::rectangle(vis, metrics.roi_box, cv::Scalar(255,255,255), 2);
            cv::putText(vis, std::string("State:") + (state==State::FORWARD?"FORWARD":state==State::SWEEP?"SWEEP":"FLIP"),
                        cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 2);
            cv::imshow("depth", vis);
            if (cv::waitKey(1) == 'q') break;
        }
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    det.stop();
    ctrl.stop();
    cv::destroyAllWindows();
    return 0;
}
