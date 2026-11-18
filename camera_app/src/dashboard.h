#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "opencv2/core.hpp"

struct DashboardTarget
{
    int track_id;
    int robot_id;
    float confidence;
    float robot_confidence;
    cv::Rect2f box;
    cv::Point2f pixel;
    cv::Point2f field;
    bool has_field_position;
};

class DashboardPublisher
{
    std::filesystem::path directory;
    std::string source;
    float field_width;
    float field_height;
    uint64_t sequence=0;
    std::chrono::steady_clock::time_point last_publish{};
    bool warned=false;

public:
    DashboardPublisher(std::string output_directory, std::string source_name, float width, float height);
    void publish(const cv::Mat& frame, const std::vector<DashboardTarget>& targets,
                 unsigned int latency_ms, unsigned int result_age_ms, bool calibrated);
};
