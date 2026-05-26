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
    struct Pt { float c, r, iz; };
    std::vector<Pt> pts;
    pts.reserve(roi_m.rows * roi_m.cols);

    for (int r = 0; r < roi_m.rows; ++r) {
        const float* pr = roi_m.ptr<float>(r);
        for (int c = 0; c < roi_m.cols; ++c) {
            float v = pr[c];
            if (!is_valid_depth(v)) continue;
            // 我们在伪3D空间运算，存储屏幕坐标(c, r)与 反转深度(1/v)
            pts.push_back({(float)c, (float)r, 1.0f / v});
        }
    }
    if (pts.empty()) return {false, m};

    // ====== 基于 RANSAC 地面识别与滤除 ======
    int best_inliers = 0;
    float best_A = 0, best_B = 0, best_C = 0;

    if (pts.size() > 50) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dis(0, pts.size() - 1);
        
        // 抽取 60 组进行平面三角测算
        for (int i = 0; i < 60; ++i) {
            const auto& p1 = pts[dis(gen)];
            const auto& p2 = pts[dis(gen)];
            const auto& p3 = pts[dis(gen)];
            
            // 叉乘计算三点形成的平面法向量 (Nc, Nr, Nz)
            float v1c = p2.c - p1.c, v1r = p2.r - p1.r, v1z = p2.iz - p1.iz;
            float v2c = p3.c - p1.c, v2r = p3.r - p1.r, v2z = p3.iz - p1.iz;
            float Nc = v1r * v2z - v1z * v2r;
            float Nr = v1z * v2c - v1c * v2z;
            float Nz = v1c * v2r - v1r * v2c;
            
            if (std::abs(Nz) < 1e-5f) continue;
            float A = -Nc / Nz;
            float B = -Nr / Nz;
            float C = p1.iz - A * p1.c - B * p1.r;
            
            // 地面的物理常识约束：视角越往下（行数 r 变大，靠近画面底边），离机器人的距离应该越近（反向深度 iz 变大）。
            // 如果 B < 0.001，意味着这是一面竖直的墙或者悬空的物体面，绝对不是地面！跳过！
            if (B < 0.001f) continue; 
            
            int inliers = 0;
            for (const auto& p : pts) {
                float exp_iz = A * p.c + B * p.r + C;
                // 用 inverse_depth 过滤，0.5f相当于在一米处容忍50cm误差，在0.5m容忍10cm上下波动
                if (std::abs(p.iz - exp_iz) < 0.5f) inliers++;
            }
            
            if (inliers > best_inliers) {
                best_inliers = inliers;
                best_A = A; best_B = B; best_C = C;
            }
        }
    }
    
    // 如果有超过 15% 的像素点构成了一个完美的“左高右低”平面，我们就断定捕捉到了地面
    bool found_ground = (best_inliers > pts.size() * 0.15f); 
    
    std::vector<float> vals;
    vals.reserve(pts.size());
    size_t close_count = 0;

    // 清洗像素：如果确定有地面，删除所有属于地面的点
    for (const auto& p : pts) {
        if (found_ground) {
            float exp_iz = best_A * p.c + best_B * p.r + best_C;
            // 误差在容忍范围内，认为是地板，扔掉！
            if (std::abs(p.iz - exp_iz) < 0.5f) continue; 
        }
        float z = 1.0f / p.iz;
        vals.push_back(z);
        if (z < detect_dist) close_count++;
    }
    
    // 如果全被当成地面过滤光了（面前一片平坦），直接返回无障碍物
    if (vals.empty()) return {false, m};

    // ====== 对剩下的“非地面纯障碍物像素”做特征计算 ======
    m.valid_ratio = static_cast<double>(vals.size()) / static_cast<double>(roi_m.rows * roi_m.cols);
    m.close_ratio = static_cast<double>(close_count) / static_cast<double>(vals.size());

    std::sort(vals.begin(), vals.end());
    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    m.mean_depth = sum / vals.size();
    m.min_depth = percentile_sorted(vals, 0.02);
    m.p10_depth = percentile_sorted(vals, 0.10);
    m.median_depth = percentile_sorted(vals, 0.50);

    m.vertical_step = 0.0; // 地面都被删了，就不用再算什么阶跃了！
    m.var_vertical = 0.0;

    // 既然地面已经被剔除了，只要这片区域还有2%的未知残留（凸起的台阶立面或者其它障碍），
    // 并且中位数或者排在最前面的距离侵入了警报线，统统当作爬楼障碍处理！
    const bool enough_obstacle = m.valid_ratio > 0.02;
    const bool close_surface = (m.median_depth < detect_dist) ||
                               (m.p10_depth < detect_dist * 0.85);

    bool is_obstacle = enough_obstacle && close_surface;
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
