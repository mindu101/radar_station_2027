// 监听 UDP 输入并打印融合结果。
//
// 输入协议（UTF-8 文本，每行一条，一个 datagram 可以带多行）：
//   L <id> <x> <y> <z> <vx> <vy> <vz> <predicted>   # 激光，ws_scan 系，米
//   C <label> <x> <y> <confidence>                  # 相机，场地系，米
//   R <slot> <x_cm> <y_cm> <valid>                  # 无线电，场地系，厘米
//
// 输出每 100ms 打印一次当前融合结果。

#include "fusion/fusion.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int)
{
    stop_requested = 1;
}

void usage(const char* program)
{
    std::printf(
        "用法: %s [选项]\n"
        "  --port N      UDP 端口，默认 11411\n"
        "  --calib FILE  calibrate_field.py 生成的激光->场地 JSON\n"
        "  --config FILE 融合参数 key=value 文件\n"
        "  --quiet       不打印每帧结果，只打印启动信息\n",
        program);
}

bool parse_int(const char* text, int& value)
{
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0')
        return false;
    value = static_cast<int>(parsed);
    return true;
}

void print_targets(const std::vector<fusion::FusedTarget>& targets, double now)
{
    std::printf("t=%.1fs targets=%zu\n", now, targets.size());
    for (const fusion::FusedTarget& target : targets)
    {
        if (target.label >= 0)
            std::printf("  label=%3d lidar=%2d pos=(%6.2f,%6.2f) v=(%5.2f,%5.2f) "
                        "src=%-9s conf=%.2f pred=%d age=%.2fs\n",
                        target.label, target.lidar_id, target.position.x,
                        target.position.y, target.velocity.x, target.velocity.y,
                        fusion::source_name(target.source), target.confidence,
                        target.predicted ? 1 : 0, target.age);
        else if (target.slot >= 0)
            std::printf("  slot=%2d pos=(%6.2f,%6.2f) src=%-9s conf=%.2f age=%.2fs\n",
                        target.slot, target.position.x, target.position.y,
                        fusion::source_name(target.source), target.confidence,
                        target.age);
        else
            std::printf("  label=  ? lidar=%2d pos=(%6.2f,%6.2f) v=(%5.2f,%5.2f) "
                        "src=%-9s conf=%.2f pred=%d age=%.2fs\n",
                        target.lidar_id, target.position.x, target.position.y,
                        target.velocity.x, target.velocity.y,
                        fusion::source_name(target.source), target.confidence,
                        target.predicted ? 1 : 0, target.age);
    }
}

} // namespace

int main(int argc, char** argv)
{
    // 重定向到日志时也按行刷新，方便 run.sh demo 直接 tail。
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    int port = 11411;
    std::string calibration_path;
    std::string config_path;
    bool quiet = false;
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
            return 0;
        }
        else if (arg == "--port")
        {
            if (!parse_int(next("--port"), port) || port <= 0 || port > 65535)
            {
                std::fprintf(stderr, "无效端口\n");
                return 2;
            }
        }
        else if (arg == "--calib")
            calibration_path = next("--calib");
        else if (arg == "--config")
            config_path = next("--config");
        else if (arg == "--quiet")
            quiet = true;
        else
        {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            usage(argv[0]);
            return 2;
        }
    }

    fusion::Fusion fusion;
    std::string error;
    if (!config_path.empty() && !fusion.config().load(config_path, &error))
    {
        std::fprintf(stderr, "加载配置失败: %s\n", error.c_str());
        return 1;
    }
    if (!calibration_path.empty() &&
        !fusion.transform().load_json(calibration_path, &error))
    {
        std::fprintf(stderr, "加载外参失败: %s\n", error.c_str());
        return 1;
    }
    if (calibration_path.empty())
        std::fprintf(stderr, "警告: 未提供 --calib，激光按与场地同坐标系处理\n");

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::perror("socket");
        return 1;
    }
    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        std::perror("bind");
        ::close(fd);
        return 1;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::printf("fusion UDP listener on 0.0.0.0:%d\n", port);

    char buffer[65536];
    double next_print = fusion::now_seconds();
    while (!stop_requested)
    {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(fd, &read_set);
        timeval timeout{};
        timeout.tv_usec = 100000;
        const int ready = ::select(fd + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready < 0)
        {
            if (errno == EINTR)
                continue;
            std::perror("select");
            break;
        }
        const double now = fusion::now_seconds();
        if (ready > 0)
        {
            const ssize_t size = ::recv(fd, buffer, sizeof(buffer) - 1, 0);
            if (size > 0)
            {
                buffer[size] = '\0';
                std::vector<fusion::LidarTrack> lidar;
                std::vector<fusion::CameraTarget> camera;
                std::vector<fusion::RadioTarget> radio;
                std::istringstream lines(buffer);
                std::string line;
                while (std::getline(lines, line))
                {
                    std::istringstream fields(line);
                    char type = '\0';
                    fields >> type;
                    if (type == 'L')
                    {
                        fusion::LidarTrack track;
                        int predicted = 0;
                        if (fields >> track.id >> track.position.x >> track.position.y >>
                                track.position.z >> track.velocity.x >> track.velocity.y >>
                                track.velocity.z >> predicted)
                        {
                            track.predicted = predicted != 0;
                            lidar.push_back(track);
                        }
                    }
                    else if (type == 'C')
                    {
                        fusion::CameraTarget target;
                        if (fields >> target.label >> target.position.x >>
                            target.position.y >> target.confidence)
                            camera.push_back(target);
                    }
                    else if (type == 'R')
                    {
                        fusion::RadioTarget target;
                        int valid = 0;
                        double x_cm = 0.0;
                        double y_cm = 0.0;
                        if (fields >> target.slot >> x_cm >> y_cm >> valid)
                        {
                            target.position = {x_cm / 100.0, y_cm / 100.0};
                            target.valid = valid != 0;
                            radio.push_back(target);
                        }
                    }
                }
                if (!lidar.empty())
                    fusion.update_lidar(lidar, now);
                if (!camera.empty())
                    fusion.update_camera(camera, now);
                if (!radio.empty())
                    fusion.update_radio(radio, now);
            }
        }
        if (now >= next_print)
        {
            next_print = now + 0.1;
            if (!quiet)
                print_targets(fusion.fuse(now), now);
        }
    }
    ::close(fd);
    return 0;
}
