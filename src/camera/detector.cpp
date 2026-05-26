#include "detector.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>
#include <random>

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
    
    // 动态获取当前分辨率下、被对齐目标(通常是彩色图)的真实内参
    rs2_stream target_stream = use_align_ ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH;
    auto stream_prof = prof.get_stream(target_stream).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = stream_prof.get_intrinsics();
    fy_ = intr.fy;
    cy_ = intr.ppy; // ppy对应的就是y方向的光心坐标
    std::cout << "[Camera Info] Resolution: " << intr.width << "x" << intr.height 
              << ", fy: " << fy_ << ", cy: " << cy_ << std::endl;

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

    // 不要再写死内参，而是使用我们在构造函数里从相机硬件动态读取的 fy_ 和 cy_
    float optical_cy = cy_; 
    float fy = fy_;         
    int roi_y_offset = m.roi_box.y; // ROI 相对于全图的行偏移

    // 简单粗暴的物理滤除：只保留 y <= 10cm 的点云，地面的物理高度通常 y > 10cm（在相机下方）
    for (int r = 0; r < roi_m.rows; ++r) {
        const float* pr = roi_m.ptr<float>(r);
        float actual_r = roi_y_offset + r;
        
        for (int c = 0; c < roi_m.cols; ++c) {
            float Z = pr[c];
            if (!is_valid_depth(Z)) continue;

            // 根据针孔相机模型计算实际的物理 Y 坐标 (向下为正)
            float physical_Y = (actual_r - optical_cy) * Z / fy;

            // 地面距离相机的垂直落差如果超过10cm (0.1m)，则剔除该点
            // y > 0.10m 意味着点云在镜头正中心下方10厘米以上的位置
            if (physical_Y > 0.10f) {
                continue; // 滤除地面点
            }
            
            // 剩下的视为有效障碍物点
            vals.push_back(Z);
        }
    }

    if (vals.empty()) return {false, m};

    // ====== 对剩下的障碍物像素计算简单的均值 ======
    m.valid_ratio = static_cast<double>(vals.size()) / static_cast<double>(roi_m.rows * roi_m.cols);
    
    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    m.mean_depth = sum / vals.size(); // 直接用均值作为障碍物距离
    
    // 为了兼容旧代码变量（比如test_detector），把其他指标都统一成均值
    m.min_depth = m.mean_depth;     
    m.p10_depth = m.mean_depth; 
    m.median_depth = m.mean_depth;

    m.vertical_step = 0.0;
    m.var_vertical = 0.0;
    m.close_ratio = (m.mean_depth < detect_dist) ? 1.0 : 0.0;

    // 当剩余有足够的点，且均值距离进入探测范围，触发判定
    const bool enough_obstacle = m.valid_ratio > 0.02; // 残留特征大于 2%
    bool is_obstacle = enough_obstacle && (m.mean_depth <= detect_dist);

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
