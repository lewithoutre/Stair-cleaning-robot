#include "detector.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace {

constexpr float kMinValidDepthM = 0.08f;
constexpr float kMaxValidDepthM = 4.0f;

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
    auto dev = prof.get_device();
    if (dev && dev.query_sensors().size() > 0) {
        try {
            auto ds = dev.first<rs2::depth_sensor>();
            depth_scale_ = ds.get_depth_scale();
        } catch(...) {}
    }
}

RealSenseStairDetector::~RealSenseStairDetector() { stop(); }

bool RealSenseStairDetector::get_frames(cv::Mat &depth_u16, cv::Mat &color_bgr) {
    rs2::frameset frames;
    if (!pipe_.poll_for_frames(&frames)) return false;
    if (use_align_) frames = align_.process(frames);

    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color = frames.get_color_frame();
    if (!depth) return false;

    depth_u16 = cv::Mat(cv::Size(depth.get_width(), depth.get_height()), CV_16U,
                        (void*)depth.get_data(), cv::Mat::AUTO_STEP).clone();

    if (color) {
        color_bgr = cv::Mat(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                            (void*)color.get_data(), cv::Mat::AUTO_STEP).clone();
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
    DetectMetrics m;
    if (depth_u16.empty()) return {false, m};
    int h = depth_u16.rows;
    int w = depth_u16.cols;
    int rw = std::clamp(static_cast<int>(w * roi_width_ratio), 1, w);
    int rh = std::clamp(static_cast<int>(h * roi_height_ratio), 1, h);
    int cx = w / 2;
    int cy = static_cast<int>(h * (1.0 - roi_height_ratio / 2.0));

    int x1 = std::max(0, cx - rw / 2);
    int x2 = std::min(w, cx + rw / 2);
    int y1 = std::max(0, cy - rh / 2);
    int y2 = std::min(h, cy + rh / 2);
    m.roi_box = cv::Rect(x1, y1, x2 - x1, y2 - y1);

    cv::Mat roi = depth_u16(m.roi_box);
    if (roi.empty()) return {false, m};

    cv::Mat roi_m = depth_to_meters(roi);
    std::vector<float> vals;
    vals.reserve(roi_m.rows * roi_m.cols);
    size_t close_count = 0;
    for (int r = 0; r < roi_m.rows; ++r) {
        const float* pr = roi_m.ptr<float>(r);
        for (int c = 0; c < roi_m.cols; ++c) {
            float v = pr[c];
            if (!is_valid_depth(v)) continue;
            vals.push_back(v);
            if (v < detect_dist) {
                ++close_count;
            }
        }
    }
    if (vals.empty()) return {false, m};

    m.valid_ratio = static_cast<double>(vals.size()) / static_cast<double>(roi_m.rows * roi_m.cols);
    m.close_ratio = static_cast<double>(close_count) / static_cast<double>(vals.size());

    std::sort(vals.begin(), vals.end());
    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    m.mean_depth = sum / vals.size();
    m.min_depth = percentile_sorted(vals, 0.02);
    m.p10_depth = percentile_sorted(vals, 0.10);
    m.median_depth = percentile_sorted(vals, 0.50);

    std::vector<float> gvals;
    gvals.reserve(static_cast<size_t>(std::max(0, roi_m.rows - 1)) * roi_m.cols);
    for (int r = 1; r < roi_m.rows; ++r) {
        const float* prev = roi_m.ptr<float>(r - 1);
        const float* cur = roi_m.ptr<float>(r);
        for (int c = 0; c < roi_m.cols; ++c) {
            if (!is_valid_depth(prev[c]) || !is_valid_depth(cur[c])) continue;
            gvals.push_back(std::abs(cur[c] - prev[c]));
        }
    }
    if (!gvals.empty()) {
        std::sort(gvals.begin(), gvals.end());
        m.vertical_step = percentile_sorted(gvals, 0.95);
        double mean_g = std::accumulate(gvals.begin(), gvals.end(), 0.0) / gvals.size();
        double var = 0.0;
        for (double v : gvals) var += (v - mean_g)*(v - mean_g);
        var /= gvals.size();
        m.var_vertical = var;
    } else {
        m.var_vertical = 0.0;
    }

    const bool enough_depth = m.valid_ratio > 0.20;
    const bool close_surface = (m.median_depth < detect_dist) ||
                               (m.p10_depth < detect_dist * 0.85) ||
                               (m.close_ratio > 0.18);
    const bool depth_discontinuity = m.vertical_step > step_variance_thresh;
    bool is_obstacle = enough_depth && (close_surface || depth_discontinuity);
    return {is_obstacle, m};
}

cv::Mat RealSenseStairDetector::colorize_depth(const cv::Mat &depth_u16, double clip_max) const {
    if (depth_u16.empty()) return {};
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
    try { pipe_.stop(); } catch(...) {}
}
