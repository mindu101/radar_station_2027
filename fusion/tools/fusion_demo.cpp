// 合成场景演示：4 个目标在场地里绕圈运动。
// - 相机每帧给出“类别 + 场地坐标”，带噪声和随机遮挡；
// - 激光每帧给出“轨迹号 + 位置 + 速度”，同样带噪声和遮挡；
// - 激光数据在 ws_scan 系，经 FieldTransform 转到场地系；
// - 融合输出与真值比较，打印每个类别的 RMSE 和遮挡期间的连续性。

#include "fusion/fusion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr double kFieldLength = 28.0;
constexpr double kFieldWidth = 15.0;

struct Robot
{
    int label = -1;
    double center_x = 0.0;
    double center_y = 0.0;
    double radius = 0.0;
    double omega = 0.0;
    double phase = 0.0;
};

struct Truth
{
    fusion::Vec2 position;
    fusion::Vec2 velocity;
};

Truth truth_at(const Robot& robot, double t)
{
    const double angle = robot.omega * t + robot.phase;
    Truth truth;
    truth.position = {robot.center_x + robot.radius * std::cos(angle),
                      robot.center_y + robot.radius * std::sin(angle)};
    truth.velocity = {-robot.radius * robot.omega * std::sin(angle),
                      robot.radius * robot.omega * std::cos(angle)};
    return truth;
}

struct Options
{
    double seconds = 12.0;
    double camera_rate = 30.0;
    double lidar_rate = 10.0;
    double camera_noise = 0.08;
    double lidar_noise = 0.05;
    double velocity_noise = 0.04;
    unsigned int seed = 7;
    bool quiet = false;
};

void usage(const char* program)
{
    std::printf(
        "用法: %s [选项]\n"
        "  --seconds N        仿真时长，默认 12\n"
        "  --camera-rate N    相机频率，默认 30\n"
        "  --lidar-rate N     激光跟踪频率，默认 10\n"
        "  --camera-noise M   相机位置噪声标准差（米），默认 0.08\n"
        "  --lidar-noise M    激光位置噪声标准差（米），默认 0.05\n"
        "  --velocity-noise M 激光速度噪声标准差（米/秒），默认 0.04\n"
        "  --seed N           随机种子，默认 7\n"
        "  --quiet            只打印汇总\n",
        program);
}

bool parse_double(const char* value, double& output)
{
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(parsed))
        return false;
    output = parsed;
    return true;
}

bool parse_options(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> const char*
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "%s 需要参数\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h")
        {
            usage(argv[0]);
            std::exit(0);
        }
        else if (arg == "--seconds")
        {
            if (!parse_double(next("--seconds"), options.seconds) || options.seconds <= 0)
                return false;
        }
        else if (arg == "--camera-rate")
        {
            if (!parse_double(next("--camera-rate"), options.camera_rate) || options.camera_rate <= 0)
                return false;
        }
        else if (arg == "--lidar-rate")
        {
            if (!parse_double(next("--lidar-rate"), options.lidar_rate) || options.lidar_rate <= 0)
                return false;
        }
        else if (arg == "--camera-noise")
        {
            if (!parse_double(next("--camera-noise"), options.camera_noise) || options.camera_noise < 0)
                return false;
        }
        else if (arg == "--lidar-noise")
        {
            if (!parse_double(next("--lidar-noise"), options.lidar_noise) || options.lidar_noise < 0)
                return false;
        }
        else if (arg == "--velocity-noise")
        {
            if (!parse_double(next("--velocity-noise"), options.velocity_noise) || options.velocity_noise < 0)
                return false;
        }
        else if (arg == "--seed")
        {
            char* end = nullptr;
            const unsigned long value = std::strtoul(next("--seed"), &end, 10);
            if (end == argv[i] || *end != '\0')
                return false;
            options.seed = static_cast<unsigned int>(value);
        }
        else if (arg == "--quiet")
        {
            options.quiet = true;
        }
        else
        {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            return false;
        }
    }
    return true;
}

// 激光雷达系相对场地系的已知外参：绕 z 转 0.4 rad，平移 (2, -1.5, 0.3)。
fusion::FieldTransform make_transform()
{
    fusion::FieldTransform transform;
    const double angle = 0.4;
    const double cos = std::cos(angle);
    const double sin = std::sin(angle);
    transform.rotation[0] = cos;
    transform.rotation[1] = -sin;
    transform.rotation[2] = 0.0;
    transform.rotation[3] = sin;
    transform.rotation[4] = cos;
    transform.rotation[5] = 0.0;
    transform.rotation[6] = 0.0;
    transform.rotation[7] = 0.0;
    transform.rotation[8] = 1.0;
    transform.translation[0] = 2.0;
    transform.translation[1] = -1.5;
    transform.translation[2] = 0.3;
    return transform;
}

struct Metrics
{
    long frames = 0;
    long outputs = 0;
    double squared_error = 0.0;
    long dropout_frames = 0;
    long dropout_outputs = 0;
    double dropout_squared_error = 0.0;
};

} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse_options(argc, argv, options))
    {
        usage(argv[0]);
        return 2;
    }

    std::mt19937 rng(options.seed);
    std::normal_distribution<double> camera_noise(0.0, options.camera_noise);
    std::normal_distribution<double> lidar_noise(0.0, options.lidar_noise);
    std::normal_distribution<double> velocity_noise(0.0, options.velocity_noise);

    const std::vector<Robot> robots = {
        {101, 14.0, 7.5, 4.0, 0.22, 0.0},
        {102, 13.0, 7.0, 3.0, 0.27, 1.4},
        {103, 15.0, 8.0, 3.5, 0.19, 2.8},
        {104, 14.5, 6.8, 2.5, 0.31, 4.1},
    };
    // 遮挡窗口（仿真时间，秒）：101 相机丢失，102 激光丢失。
    const double camera_dropout_begin = 4.0;
    const double camera_dropout_end = 5.5;
    const double lidar_dropout_begin = 2.0;
    const double lidar_dropout_end = 3.5;

    const fusion::FieldTransform transform = make_transform();
    fusion::Fusion fusion;
    fusion.set_transform(transform);

    const double time_base = 1000.0; // 避免出现接近 0 的时间戳
    const double dt = 1.0 / 60.0;
    const int steps = static_cast<int>(std::ceil(options.seconds / dt));

    std::map<int, Metrics> metrics;
    for (const Robot& robot : robots)
        metrics[robot.label];

    double next_camera = 0.0;
    double next_lidar = 0.0;
    int printed = 0;

    for (int step = 0; step < steps; ++step)
    {
        const double t = step * dt;
        const double now = time_base + t;
        std::map<int, Truth> truths;
        for (const Robot& robot : robots)
            truths[robot.label] = truth_at(robot, t);

        if (t + 1e-9 >= next_camera)
        {
            std::vector<fusion::CameraTarget> targets;
            for (const Robot& robot : robots)
            {
                if (robot.label == 101 && t >= camera_dropout_begin && t < camera_dropout_end)
                    continue;
                const Truth& truth = truths[robot.label];
                targets.push_back({robot.label,
                                   {truth.position.x + camera_noise(rng),
                                    truth.position.y + camera_noise(rng)},
                                   1.0F});
            }
            fusion.update_camera(targets, now);
            next_camera += 1.0 / options.camera_rate;
        }

        if (t + 1e-9 >= next_lidar)
        {
            std::vector<fusion::LidarTrack> tracks;
            for (std::size_t i = 0; i < robots.size(); ++i)
            {
                const Robot& robot = robots[i];
                if (robot.label == 102 && t >= lidar_dropout_begin && t < lidar_dropout_end)
                    continue;
                const Truth& truth = truths[robot.label];
                // field -> lidar：p_lidar = R^T * (p_field - t)
                const fusion::Vec3 field{truth.position.x, truth.position.y, 0.0};
                const fusion::Vec3 shifted{field.x - transform.translation[0],
                                           field.y - transform.translation[1],
                                           field.z - transform.translation[2]};
                fusion::Vec3 lidar_position;
                lidar_position.x = transform.rotation[0] * shifted.x +
                                   transform.rotation[3] * shifted.y +
                                   transform.rotation[6] * shifted.z;
                lidar_position.y = transform.rotation[1] * shifted.x +
                                   transform.rotation[4] * shifted.y +
                                   transform.rotation[7] * shifted.z;
                lidar_position.z = transform.rotation[2] * shifted.x +
                                   transform.rotation[5] * shifted.y +
                                   transform.rotation[8] * shifted.z;
                lidar_position.x += lidar_noise(rng);
                lidar_position.y += lidar_noise(rng);

                fusion::Vec3 lidar_velocity;
                lidar_velocity.x = transform.rotation[0] * truth.velocity.x +
                                   transform.rotation[3] * truth.velocity.y;
                lidar_velocity.y = transform.rotation[1] * truth.velocity.x +
                                   transform.rotation[4] * truth.velocity.y;
                lidar_velocity.z = 0.0;
                lidar_velocity.x += velocity_noise(rng);
                lidar_velocity.y += velocity_noise(rng);

                tracks.push_back({static_cast<int>(i) + 1, lidar_position,
                                  lidar_velocity, false});
            }
            fusion.update_lidar(tracks, now);
            next_lidar += 1.0 / options.lidar_rate;
        }

        const std::vector<fusion::FusedTarget> fused = fusion.fuse(now);
        for (const Robot& robot : robots)
        {
            Metrics& metric = metrics[robot.label];
            ++metric.frames;
            const Truth& truth = truths[robot.label];
            bool in_camera_dropout =
                robot.label == 101 && t >= camera_dropout_begin && t < camera_dropout_end;
            if (in_camera_dropout)
                ++metric.dropout_frames;

            for (const fusion::FusedTarget& target : fused)
            {
                if (target.label != robot.label)
                    continue;
                const double error = std::hypot(target.position.x - truth.position.x,
                                                target.position.y - truth.position.y);
                ++metric.outputs;
                metric.squared_error += error * error;
                if (in_camera_dropout)
                {
                    ++metric.dropout_outputs;
                    metric.dropout_squared_error += error * error;
                }
                if (!options.quiet && printed < 8 && step % 60 == 0)
                {
                    std::printf("t=%5.2fs label=%3d lidar=%2d pos=(%6.2f,%6.2f) "
                                "v=(%5.2f,%5.2f) src=%-9s conf=%.2f pred=%d\n",
                                t, target.label, target.lidar_id, target.position.x,
                                target.position.y, target.velocity.x, target.velocity.y,
                                fusion::source_name(target.source), target.confidence,
                                target.predicted ? 1 : 0);
                    ++printed;
                }
                break;
            }
        }
    }

    std::printf("\n仿真 %.1fs，相机 %.0fHz，激光 %.0fHz，场地 %.0fx%.0f m\n",
                options.seconds, options.camera_rate, options.lidar_rate,
                kFieldLength, kFieldWidth);
    std::printf("%-6s %8s %8s %10s %10s\n", "label", "输出率", "RMSE(m)", "遮挡帧", "遮挡输出率");
    for (const Robot& robot : robots)
    {
        const Metrics& metric = metrics.at(robot.label);
        const double rate = metric.frames ? 100.0 * metric.outputs / metric.frames : 0.0;
        const double rmse = metric.outputs ? std::sqrt(metric.squared_error / metric.outputs) : 0.0;
        if (metric.dropout_frames > 0)
        {
            const double dropout_rate =
                100.0 * metric.dropout_outputs / metric.dropout_frames;
            const double dropout_rmse =
                metric.dropout_outputs
                    ? std::sqrt(metric.dropout_squared_error / metric.dropout_outputs)
                    : 0.0;
            std::printf("%-6d %7.1f%% %8.3f %6ld(%4.1fs) %7.1f%% (RMSE %.3f)\n",
                        robot.label, rate, rmse, metric.dropout_frames,
                        metric.dropout_frames * dt, dropout_rate, dropout_rmse);
        }
        else
        {
            std::printf("%-6d %7.1f%% %8.3f %10s %10s\n",
                        robot.label, rate, rmse, "-", "-");
        }
    }
    return 0;
}
