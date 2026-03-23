#include "detector.h"
#include <numeric>

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
    int rw = static_cast<int>(w * roi_width_ratio);
    int rh = static_cast<int>(h * roi_height_ratio);
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
    cv::Mat valid_mask = roi_m > 0.0f;
    if (cv::countNonZero(valid_mask) == 0) return {false, m};

    // compute mean and min over valid
    cv::Mat valid_vals;
    roi_m.copyTo(valid_vals, valid_mask);
    std::vector<float> vals;
    vals.reserve(roi_m.rows * roi_m.cols);
    for (int r = 0; r < valid_vals.rows; ++r) {
        const float* pr = valid_vals.ptr<float>(r);
        for (int c = 0; c < valid_vals.cols; ++c) {
            float v = pr[c];
            if (v > 0) vals.push_back(v);
        }
    }
    if (vals.empty()) return {false, m};
    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    m.mean_depth = sum / vals.size();
    m.min_depth = *std::min_element(vals.begin(), vals.end());

    // vertical gradient variance
    cv::Mat diff;
    cv::absdiff(roi_m.rowRange(1, roi_m.rows), roi_m.rowRange(0, roi_m.rows-1), diff);
    cv::Mat diff_mask = diff > 0.0f;
    std::vector<float> gvals;
    for (int r = 0; r < diff.rows; ++r) {
        const float* pr = diff.ptr<float>(r);
        const uchar* pm = diff_mask.ptr<uchar>(r);
        for (int c = 0; c < diff.cols; ++c) {
            if (pm[c]) gvals.push_back(pr[c]);
        }
    }
    if (!gvals.empty()) {
        double mean_g = std::accumulate(gvals.begin(), gvals.end(), 0.0) / gvals.size();
        double var = 0.0;
        for (double v : gvals) var += (v - mean_g)*(v - mean_g);
        var /= gvals.size();
        m.var_vertical = var;
    } else {
        m.var_vertical = 0.0;
    }

    bool is_obstacle = (m.mean_depth < detect_dist) || (m.min_depth < detect_dist * 0.8) || (m.var_vertical > step_variance_thresh);
    return {is_obstacle, m};
}

cv::Mat RealSenseStairDetector::colorize_depth(const cv::Mat &depth_u16, double clip_max) const {
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
