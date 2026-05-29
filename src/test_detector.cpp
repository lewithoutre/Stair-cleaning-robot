#include <iomanip>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "detector.h"

int main() {
    std::cout << "=========================================" << std::endl;
    std::cout << "  RealSense Stair Detector Test Tool     " << std::endl;
    std::cout << "  Press 'q' or 'ESC' on image window to quit." << std::endl;
    std::cout << "=========================================" << std::endl;

    // 参数：分辨率 640x480，深度对齐至彩色图
    RealSenseStairDetector detector(640, 480, true);

    cv::Mat depth;
    cv::Mat color;

    while (true) {
        // 读取画面
        if (!detector.get_frames(depth, color)) {
            continue;
        }

        // 调用判定函数。
        // 默认参数：
        // roi_height_ratio = 0.35
        // roi_width_ratio  = 0.4
        // detect_dist      = 0.6m
        auto [is_obstacle, metrics] = detector.detect_obstacle(depth);

        // 当前 detector.cpp 的触发条件为：
        // 滤除相机下方超过 0.1m 的地面点后，
        // 剩余点云的 10% 分位距离 p10_depth 进入阈值。
        const bool close_surface = metrics.p10_depth > 0.0 &&
                                   metrics.p10_depth < 0.6;

        const bool depth_discontinuity = metrics.vertical_step > 0.12;

        // 控制台打印实时测算数据
        std::cout << "\r[ "
                  << (is_obstacle ? "CLIMB MODE TRIGGERED" : "FORWARD (NO OBSTACLE)")
                  << " ]"
                  << " | Front Dist(p10): "
                  << std::fixed << std::setprecision(2)
                  << metrics.p10_depth
                  << "m (Near threshold=" << (close_surface ? "Yes" : "No") << ")"
                  << " | Min Dist: "
                  << std::fixed << std::setprecision(2)
                  << metrics.min_depth
                  << "m"
                  << " | Step Drop: "
                  << metrics.vertical_step
                  << "m (Step threshold=" << (depth_discontinuity ? "Yes" : "No") << ")"
                  << " | Valid Pixels: "
                  << static_cast<int>(metrics.valid_ratio * 100)
                  << "%    "
                  << std::flush;

        // 图像可视化
        cv::Mat colorized_depth = detector.colorize_depth(depth, 2.0);

        // 用一个框把 ROI 探测区域画出来。
        // 绿色代表未触发，红色代表检测到障碍物。
        cv::Scalar box_color = is_obstacle
            ? cv::Scalar(0, 0, 255)
            : cv::Scalar(0, 255, 0);

        if (!colorized_depth.empty() && metrics.roi_box.area() > 0) {
            cv::rectangle(colorized_depth, metrics.roi_box, box_color, 2);
            cv::imshow("Depth View (Test)", colorized_depth);
        }

        if (!color.empty()) {
            if (metrics.roi_box.area() > 0) {
                cv::rectangle(color, metrics.roi_box, box_color, 2);

                std::string text = is_obstacle
                    ? "ACTION: TRIGGER CLIMB"
                    : "ACTION: KEEP FORWARD";

                cv::putText(color,
                            text,
                            cv::Point(metrics.roi_box.x, metrics.roi_box.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            box_color,
                            2);

                int text_y = 30;

                cv::putText(color,
                            "Front Dist(p10): " +
                                std::to_string(metrics.p10_depth).substr(0, 4) +
                                " m",
                            cv::Point(10, text_y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            cv::Scalar(255, 255, 255),
                            2);
                text_y += 25;

                cv::putText(color,
                            "Min Dist: " +
                                std::to_string(metrics.min_depth).substr(0, 4) +
                                " m",
                            cv::Point(10, text_y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            cv::Scalar(255, 255, 255),
                            2);
                text_y += 25;

                cv::putText(color,
                            "Is Too Close? " +
                                std::string(close_surface ? "YES" : "NO"),
                            cv::Point(10, text_y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            close_surface
                                ? cv::Scalar(0, 0, 255)
                                : cv::Scalar(0, 255, 0),
                            2);
                text_y += 25;

                cv::putText(color,
                            "Step Drop Height: " +
                                std::to_string(metrics.vertical_step).substr(0, 4) +
                                " m",
                            cv::Point(10, text_y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            cv::Scalar(255, 255, 255),
                            2);
                text_y += 25;

                cv::putText(color,
                            "Has Stair Feature? " +
                                std::string(depth_discontinuity ? "YES" : "NO"),
                            cv::Point(10, text_y),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.6,
                            depth_discontinuity
                                ? cv::Scalar(0, 0, 255)
                                : cv::Scalar(0, 255, 0),
                            2);
            }

            cv::imshow("Color View (Test)", color);
        }

        const int key = cv::waitKey(30);
        if (key == 27 || key == 'q') {
            break;
        }
    }

    std::cout << "\nExiting test." << std::endl;

    detector.stop();

    return 0;
}