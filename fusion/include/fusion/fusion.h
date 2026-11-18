#ifndef RADAR_FUSION_FUSION_H
#define RADAR_FUSION_FUSION_H

#include "fusion/types.h"

#include <unordered_map>

namespace fusion {

// 激光 + 相机（+ 可选无线电）的多源位置融合。
//
// 工作方式：
//   1. 激光轨迹本身没有机器人身份，相机检测有类别但没有连续轨迹。
//      每帧用相机位置去和激光轨迹做最近邻关联（带门限和绑定保持惩罚），
//      把“类别”绑定到激光轨迹上。
//   2. 相机和激光都新鲜时，输出位置按 lidar_weight 做加权；
//      相机短时丢失时，用关联激光的外推位置继续维持身份（identity_ttl 内）；
//      只有相机时按相机速度短时外推（camera_hold 内）。
//   3. 没有相机的激光轨迹以 label=-1 输出，供调试或占用位检测。
//   4. 可选：无线电槽位位置在没有相机/激光覆盖时作为兜底。
//
// 该类不是线程安全的：所有 update_* 和 fuse 应由同一个线程调用。
class Fusion
{
public:
    Fusion() = default;
    explicit Fusion(const FusionConfig& config) : config_(config) {}

    const FieldTransform& transform() const { return transform_; }
    FieldTransform& transform() { return transform_; }
    const FusionConfig& config() const { return config_; }
    FusionConfig& config() { return config_; }

    void set_transform(const FieldTransform& transform) { transform_ = transform; }

    // 传入一帧激光跟踪结果（ws_scan 系，米），now 为单调时钟秒。
    void update_lidar(const std::vector<LidarTrack>& tracks, double now);
    // 传入一帧相机检测（场地系，米）。
    void update_camera(const std::vector<CameraTarget>& targets, double now);
    // 传入一帧无线电槽位位置（场地系，米）。
    void update_radio(const std::vector<RadioTarget>& targets, double now);

    // 计算当前融合结果。
    std::vector<FusedTarget> fuse(double now);

    void reset();

private:
    struct LidarState
    {
        Vec3 position;
        Vec3 velocity;
        double stamp = 0.0;
        bool predicted = false;
        bool has_sample = false;
    };

    struct IdentityState
    {
        Vec2 camera_position;
        Vec2 camera_velocity;
        double camera_stamp = 0.0;
        float confidence = 0.0F;
        int lidar_id = -1;
        bool has_camera_sample = false;
    };

    struct RadioState
    {
        Vec2 position;
        double stamp = 0.0;
        bool valid = false;
    };

    void prune(double now);

    FusionConfig config_;
    FieldTransform transform_;
    std::unordered_map<int, LidarState> lidar_;
    std::unordered_map<int, IdentityState> identities_;
    std::unordered_map<int, RadioState> radio_; // key: slot
};

} // namespace fusion

#endif
