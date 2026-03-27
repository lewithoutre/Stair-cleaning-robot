#pragma once

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <optional>
#include <tuple>

struct DetectMetrics {
    double mean_depth = 0.0;
    double min_depth = 0.0;
    double var_vertical = 0.0;
    cv::Rect roi_box;
};

class RealSenseStairDetector {
public:
    RealSenseStairDetector(int width = 640, int height = 480, bool align_to_color = true);
    ~RealSenseStairDetector();

    // fetch next frames; returns depth (uint16) and color (BGR) mats; depth may be empty
    bool get_frames(cv::Mat &depth_u16, cv::Mat &color_bgr);

    // convert depth (u16) to meters
    cv::Mat depth_to_meters(const cv::Mat &depth_u16) const;

    // detect obstacle in front; returns pair(is_obstacle, metrics)
    std::pair<bool, DetectMetrics> detect_obstacle(const cv::Mat &depth_u16,
                                                   double roi_height_ratio = 0.35,
                                                   double roi_width_ratio = 0.4,
                                                   double detect_dist = 0.6,
                                                   double step_variance_thresh = 0.08) const;

    cv::Mat colorize_depth(const cv::Mat &depth_u16, double clip_max = 3.0) const;

    void stop();

private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::align align_{RS2_STREAM_COLOR};
    bool use_align_ = true;
    float depth_scale_ = 0.001f;
    int width_, height_;
};
