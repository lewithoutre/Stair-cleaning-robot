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

    cv::Mat depth, color;
    
    while (true) {
        // 读取画面
        if (!detector.get_frames(depth, color)) {
            continue;
        }

        // 调用判定函数 (使用默认参数: roi_height_ratio=0.35, width_ratio=0.4, detect_dist=0.6m)
        // 这个函数就是计算是不是楼梯的核心代码
        auto [is_obstacle, metrics] = detector.detect_obstacle(depth);

        // ==== 提取导致触发爬楼的具体细分条件 ====
        // 在 detector.cpp 的源码中，触发条件分为“距离太近”或“有阶跃落差”
        bool close_surface = (metrics.median_depth < 0.6) || (metrics.p10_depth < 0.6 * 0.85) || (metrics.close_ratio > 0.18);
        bool depth_discontinuity = (metrics.vertical_step > 0.12);

        // 控制台打印实时测算数据
        std::cout << "\r[ " << (is_obstacle ? "🔴 CLIMB MODE TRIGGERED" : "🟢 FORWARD (NO OBSTACLE)") << " ]"
                  << " | Front Dist: " << std::fixed << std::setprecision(2) << metrics.p10_depth << "m (Near threshold=" << (close_surface?"Yes":"No") << ")"
                  << " | Step Drop: " << metrics.vertical_step << "m (Step threshold=" << (depth_discontinuity?"Yes":"No") << ")"
                  << " | Valid Pixels: " << static_cast<int>(metrics.valid_ratio * 100) << "%    " << std::flush;

        // 图像可视化
        cv::Mat colorized_depth = detector.colorize_depth(depth, 2.0); // 最大映射距离2.0米
        
        // 我们用一个框把 ROI 探测区域画出来 (绿色代表平坦，红色代表遇到了楼梯)
        cv::Scalar box_color = is_obstacle ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);

        if (!colorized_depth.empty() && metrics.roi_box.area() > 0) {
            cv::rectangle(colorized_depth, metrics.roi_box, box_color, 2);
            cv::imshow("Depth View (Test)", colorized_depth);
        }

        if (!color.empty()) {
            if (metrics.roi_box.area() > 0) {
                // 画边界框
                cv::rectangle(color, metrics.roi_box, box_color, 2);
                
                // 在框的上边缘写主动作提示
                std::string text = is_obstacle ? "ACTION: TRIGGER CLIMB" : "ACTION: KEEP FORWARD";
                cv::putText(color, text, cv::Point(metrics.roi_box.x, metrics.roi_box.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2);

                // === 新增：在视频左上角打上爬楼判定的核心指标水印 ===
                int text_y = 30;
                cv::putText(color, "Front Dist(p10): " + std::to_string(metrics.p10_depth).substr(0,4) + " m", 
                            cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2); text_y += 25;
                
                cv::putText(color, "Is Too Close? " + std::string(close_surface ? "YES" : "NO"), 
                            cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, close_surface ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 2); text_y += 25;

                cv::putText(color, "Step Drop Height: " + std::to_string(metrics.vertical_step).substr(0,4) + " m", 
                            cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2); text_y += 25;

                cv::putText(color, "Has Stair Feature? " + std::string(depth_discontinuity ? "YES" : "NO"), 
                            cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, depth_discontinuity ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 2);
            }
            cv::imshow("Color View (Test)", color);
        }

        // 刷新 GUI 窗口，等待按键退出
        int key = cv::waitKey(30);
        if (key == 27 || key == 'q') {
            break;
        }
    }
    
    std::cout << "\nExiting test." << std::endl;
    detector.stop();
    return 0;
}