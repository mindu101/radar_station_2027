#ifndef RADAR_FUSION_TYPES_H
#define RADAR_FUSION_TYPES_H

// 激光雷达 + 相机 + （可选）无线电的位置融合数据结构。
//
// 坐标约定：
//   - 激光雷达输入是 ws_scan 系，单位米；Fusion 内部先用 FieldTransform 转到场地系。
//   - 相机输入已经是场地系（0x0305/0x0301 使用的坐标系），单位米。
//   - 无线电输入是场地系，单位米（UDP 协议里用 cm，监听器负责换算）。
//   - 融合输出统一是场地系、米。
//
// 身份约定：
//   - 相机 label 沿用上一届装甲/车辆分类器的类别号（0~11 那种），融合层只当作
//     不透明整数，不假设具体含义。
//   - lidar_id 是激光跟踪节点自己分配的轨迹号，仅用于调试和连续性判断。
//   - slot 可选，0~5，与 robot_classes 的槽位顺序一致；无线电输入会直接带槽位。

#include <cstdint>
#include <string>
#include <vector>

namespace fusion {

struct Vec2
{
    double x = 0.0;
    double y = 0.0;
};

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 激光雷达系 -> 场地系的刚体变换，字段与 scripts/calibrate_field.py 生成的
// JSON 一致：field = R * lidar + t，rotation 按行优先存 9 个数。
struct FieldTransform
{
    double rotation[9] = {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0};
    double translation[3] = {0.0, 0.0, 0.0};

    Vec3 apply(const Vec3& lidar_point) const;
    Vec3 apply_rotation(const Vec3& lidar_vector) const;
    void set_identity();

    // 读取 calibrate_field.py 生成的 JSON（只解析 rotation/translation）。
    bool load_json(const std::string& path, std::string* error = nullptr);
};

// 一帧激光跟踪结果。position/velocity 是 ws_scan 系，单位米。
struct LidarTrack
{
    int id = -1;
    Vec3 position;
    Vec3 velocity;
    bool predicted = false;
};

// 一帧相机检测结果。position 已经是场地系，单位米。
struct CameraTarget
{
    int label = -1;
    Vec2 position;
    float confidence = 1.0F;
};

// 一帧无线电（0x0A01 / 解波）结果。position 是场地系，单位米。
struct RadioTarget
{
    int slot = -1; // 0~5，与 robot_classes 槽位一致
    Vec2 position;
    bool valid = true;
};

enum class Source : std::uint8_t
{
    None = 0,
    Camera,         // 只有相机测量
    Lidar,          // 相机 + 激光关联后的位置（或激光自身）
    LidarPredicted, // 相机丢失，靠激光轨迹延续身份
    Radio,          // 无线电兜底
};

struct FusedTarget
{
    int label = -1;   // 相机类别号；-1 表示身份未知
    int lidar_id = -1; // 关联到的激光轨迹号；-1 表示没有
    int slot = -1;     // 可选槽位；无线电输入会填，其他来源默认 -1
    Vec2 position;     // 场地系，米
    Vec2 velocity;     // 场地系，米/秒
    float confidence = 0.0F;
    Source source = Source::None;
    double age = 0.0;  // 距最近一次真实测量的时间，秒
    bool predicted = false;
};

struct FusionConfig
{
    // 相机 -> 激光 关联门限，米。
    double gate = 1.2;
    // 优先沿用上次绑定的激光轨迹，避免在两个轨迹之间来回跳。
    double switch_penalty = 0.25;
    // 相机样本在该时间内视为“新鲜”，秒。
    double camera_ttl = 0.35;
    // 只有相机时，最多外推这么久；超过就丢弃，秒。
    double camera_hold = 0.8;
    // 相机丢失后，靠激光轨迹保留身份的最长时间，秒。
    double identity_ttl = 2.0;
    // 激光轨迹多久没更新就视为失效，秒。
    double lidar_ttl = 0.6;
    // 相机和激光都新鲜时，激光位置权重：0=纯相机，1=纯激光。
    double lidar_weight = 0.7;
    // 相机速度低通系数（新测量的权重）。
    double camera_velocity_alpha = 0.35;
    // 激光速度低通系数（新测量的权重，1 表示完全相信激光节点）。
    double lidar_velocity_alpha = 1.0;
    // 是否使用无线的槽位位置做兜底。
    bool use_radio = false;
    double radio_ttl = 0.5;
    // 无线电结果与已有融合结果的距离小于该值时视为重复，米。
    double radio_gate = 2.0;
    // 是否输出没有身份（label=-1）的激光轨迹。
    bool include_unlabeled = true;
    // 低于该置信度的相机检测直接丢弃。
    float min_camera_confidence = 0.0F;

    // 读取简单的 key = value 配置文件，支持 # 注释。
    bool load(const std::string& path, std::string* error = nullptr);
};

// 单调时钟，秒。
double now_seconds();

const char* source_name(Source source);

} // namespace fusion

#endif
