#include "detector.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace {

constexpr float kMinValidDepthM = 0.01f;
constexpr float kMaxValidDepthM = 1.0f;

bool is_valid_depth(float value) {
    return std::isfinite(value) && value >= kMinValidDepthM && value <= kMaxValidDepthM;
}

double percentile_sorted(const std::vector<float>& sorted_values, double percentile) {
    if (sorted_values.empty()) return 0.0;
    if (percentile <= 0.0) return sorted_values.front();
    if (percentile >= 1.0) return sorted_values.back();

    const double index = percentile * static_cast<double>(sorted_values.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(index));
    const auto hi = static_cast<size_t>(std::ceil(index));
    const double t = index - static_cast<double>(lo);

    return sorted_values[lo] * (1.0 - t) + sorted_values[hi] * t;
}

} // namespace

RealSenseStairDetector::RealSenseStairDetector(int width, int height, bool align_to_color)
    : align_(RS2_STREAM_COLOR), use_align_(align_to_color), width_(width), height_(height)
{
    cfg_.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, 30);
    cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile prof = pipe_.start(cfg_);

    // 动态获取当前分辨率下、被对齐目标通常是彩色图的真实内参
    rs2_stream target_stream = use_align_ ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH;
    auto stream_prof = prof.get_stream(target_stream).as<rs2::video_stream_profile>();
    intr_ = stream_prof.get_intrinsics();

    std::cout << "[Camera Info] Resolution: " << intr_.width << "x" << intr_.height
              << ", fy: " << intr_.fy
              << ", cy: " << intr_.ppy
              << std::endl;

    auto dev = prof.get_device();
    if (dev && dev.query_sensors().size() > 0) {
        try {
            auto ds = dev.first<rs2::depth_sensor>();
            depth_scale_ = ds.get_depth_scale();
        } catch (...) {
        }
    }
}

RealSenseStairDetector::~RealSenseStairDetector() {
    stop();
}

bool RealSenseStairDetector::get_frames(cv::Mat &depth_u16, cv::Mat &color_bgr) {
    rs2::frameset frames;

    if (!pipe_.poll_for_frames(&frames)) {
        return false;
    }

    if (use_align_) {
        frames = align_.process(frames);
    }

    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color = frames.get_color_frame();

    if (!depth) {
        return false;
    }

    depth_u16 = cv::Mat(cv::Size(depth.get_width(), depth.get_height()),
                        CV_16U,
                        (void*)depth.get_data(),
                        cv::Mat::AUTO_STEP).clone();

    if (color) {
        color_bgr = cv::Mat(cv::Size(color.get_width(), color.get_height()),
                            CV_8UC3,
                            (void*)color.get_data(),
                            cv::Mat::AUTO_STEP).clone();
    } else {
        color_bgr.release();
    }

    return true;
}

cv::Mat RealSenseStairDetector::depth_to_meters(const cv::Mat &depth_u16) const {
    cv::Mat depth_f;
    depth_u16.convertTo(depth_f, CV_32F, depth_scale_);
    return depth_f;
}

std::pair<bool, DetectMetrics> RealSenseStairDetector::detect_obstacle(const cv::Mat &depth_u16,
                                                                       double roi_height_ratio,
                                                                       double roi_width_ratio,
                                                                       double detect_dist,
                                                                       double step_variance_thresh) const
{
    (void)step_variance_thresh;

    DetectMetrics m;

    if (depth_u16.empty()) {
        return {false, m};
    }

    const int h = depth_u16.rows;
    const int w = depth_u16.cols;

    const int rw = std::clamp(static_cast<int>(w * roi_width_ratio), 1, w);
    const int rh = std::clamp(static_cast<int>(h * roi_height_ratio), 1, h);

    const int cx = w / 2;
    const int cy = static_cast<int>(h * (1.0 - roi_height_ratio / 2.0));

    const int x1 = std::max(0, cx - rw / 2);
    const int x2 = std::min(w, cx + rw / 2);
    const int y1 = std::max(0, cy - rh / 2);
    const int y2 = std::min(h, cy + rh / 2);

    m.roi_box = cv::Rect(x1, y1, x2 - x1, y2 - y1);

    cv::Mat roi = depth_u16(m.roi_box);
    if (roi.empty()) {
        return {false, m};
    }

    cv::Mat roi_m = depth_to_meters(roi);

    std::vector<float> vals;
    vals.reserve(roi_m.rows * roi_m.cols);

    // ROI 相对于全图的偏移。反投影时必须用全图像素坐标。
    const int roi_y_offset = m.roi_box.y;
    const int roi_x_offset = m.roi_box.x;

    // 核心逻辑：
    // 1. 遍历 ROI 内所有深度点。
    // 2. 用 RealSense 内参把像素点反投影为三维点云。
    // 3. 滤除相机下方超过 0.1m 的点云，认为这些点大概率是地面。
    // 4. 剩余点云作为障碍物候选点。
    for (int r = 0; r < roi_m.rows; ++r) {
        const float* pr = roi_m.ptr<float>(r);
        const float actual_r = static_cast<float>(roi_y_offset + r);

        for (int c = 0; c < roi_m.cols; ++c) {
            const float Z = pr[c];

            if (!is_valid_depth(Z)) {
                continue;
            }

            const float actual_c = static_cast<float>(roi_x_offset + c);

            float pixel[2] = {actual_c, actual_r};
            float point[3];

            // 使用 RealSense SDK 计算三维坐标 X, Y, Z。
            rs2_deproject_pixel_to_point(point, &intr_, pixel, Z);

            const float physical_Y = point[1];

            // 地面距离相机的垂直落差如果超过 10cm，则剔除该点。
            // y > 0.10m 表示点云在镜头中心下方 10cm 以上的位置。
            if (physical_Y > 0.10f) {
                continue;
            }

            vals.push_back(Z);
        }
    }

    if (vals.empty()) {
        return {false, m};
    }

    // ====== 对剩下的障碍物候选点计算距离指标 ======
    // vals 已经滤除了相机下方超过 0.1m 的地面点。
    // 为了避免单个飞点/噪声点误触发，不直接用最近点 min_depth 作为控制距离。
    // 这里使用 10% 分位距离 p10_depth：
    // 即剩余点云中按深度排序后，靠前 10% 位置的深度值。
    m.valid_ratio = static_cast<double>(vals.size()) /
                    static_cast<double>(roi_m.rows * roi_m.cols);

    std::sort(vals.begin(), vals.end());

    m.min_depth = vals.front();
    m.p10_depth = percentile_sorted(vals, 0.10);
    m.median_depth = percentile_sorted(vals, 0.50);

    // 兼容主程序当前逻辑：
    // main.cpp 的 Approach 阶段使用 metrics.mean_depth 做距离判断。
    // 因此这里把 mean_depth 定义为主控制距离，并同步为 p10_depth。
    m.mean_depth = m.p10_depth;

    m.vertical_step = 0.0;
    m.var_vertical = 0.0;

    const auto close_count = std::count_if(
        vals.begin(),
        vals.end(),
        [detect_dist](float z) {
            return z <= detect_dist;
        }
    );

    m.close_ratio = static_cast<double>(close_count) /
                    static_cast<double>(vals.size());

    // 判定条件：
    // 1. ROI 里滤除地面后，仍有超过 2% 的有效非地面点。
    // 2. 这些点的 p10 距离小于等于 detect_dist。
    //
    // p10 比 min 更稳：
    // 少量异常近点不会直接触发，但障碍物前沿稳定出现时会触发。
    const bool enough_obstacle = m.valid_ratio > 0.02;
    const bool is_obstacle = enough_obstacle && (m.p10_depth <= detect_dist);

    return {is_obstacle, m};
}

cv::Mat RealSenseStairDetector::colorize_depth(const cv::Mat &depth_u16, double clip_max) const {
    if (depth_u16.empty()) {
        return {};
    }

    cv::Mat d_m = depth_to_meters(depth_u16);
    cv::Mat disp;

    cv::Mat norm = d_m / clip_max;
    norm.setTo(1.0, norm > 1.0);
    norm.setTo(0.0, norm < 0.0);
    norm = 1.0 - norm;
    norm.convertTo(disp, CV_8U, 255.0);

    cv::Mat color;
    cv::applyColorMap(disp, color, cv::COLORMAP_JET);

    return color;
}

void RealSenseStairDetector::stop() {
    try {
        pipe_.stop();
    } catch (...) {
    }
}