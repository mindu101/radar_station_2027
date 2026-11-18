#ifndef RADAR_FUSION_CAMERA_FUSION_H
#define RADAR_FUSION_CAMERA_FUSION_H

// 上一届相机程序的直接适配器（融合方案 A）。
//
// 相机程序只需要：
//   1. 在 output() 里把 center_points/ids 交给 update()；
//   2. 用 update() 的返回值给 msger.pos_from_camera 赋值；
//   3. 启动时调用 start_udp() 接收激光轨迹。
//
// CameraFusion 自己负责：
//   - 后台 UDP 线程接收 L/C/R 行（L 行为激光，ws_scan 系）；
//   - 把相机 ids 里的 255 哨兵（上一届用 -1 存进 uint8_t）当成“无身份”丢弃；
//   - 加锁，保证相机线程和 UDP 线程不会同时访问 Fusion；
//   - 每帧调用 update() 时即使没有检测也会推进相机丢失计时。

#include "fusion/fusion.h"

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fusion {

class CameraFusion
{
public:
    CameraFusion() = default;
    ~CameraFusion();

    CameraFusion(const CameraFusion&) = delete;
    CameraFusion& operator=(const CameraFusion&) = delete;

    bool load_calibration(const std::string& path, std::string* error = nullptr);
    bool load_config(const std::string& path, std::string* error = nullptr);
    void set_config(const FusionConfig& config);

    // 启动激光 UDP 接收线程。port 为 0 时不启动（纯相机模式）。
    bool start_udp(std::uint16_t port, std::string* error = nullptr);
    void stop();
    bool udp_running() const { return running_.load(); }

    // 相机线程每帧调用一次。
    //   points/labels 与上一届 output() 的 center_points/ids 完全一致（米、场地系）；
    //   labels 里的 255 会被丢弃；
    //   confidences 可为空，空表示全部按 1.0 处理。
    // 返回值可直接赋给 msger.pos_from_camera。
    std::pair<std::vector<cv::Point2f>, std::vector<std::uint8_t>> update(
        const std::vector<cv::Point2f>& points,
        const std::vector<std::uint8_t>& labels,
        const std::vector<float>& confidences = {});

    // 需要完整 FusedTarget（来源、速度、置信度、未标定激光轨迹）时用这个。
    std::vector<FusedTarget> fused();

private:
    void receive_loop();
    void handle_datagram(const char* buffer, std::size_t size, double now);

    mutable std::mutex mutex_;
    Fusion fusion_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    int fd_ = -1;
};

} // namespace fusion

#endif
