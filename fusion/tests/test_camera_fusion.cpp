// 相机适配器测试：重点是上一届相机程序用 255 表示“无身份”。
#include "fusion/camera_fusion.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,         \
                         __LINE__, #condition);                                 \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

} // namespace

int main()
{
    // 正常类别：返回值可以直接给 msger.pos_from_camera。
    {
        fusion::CameraFusion camera_fusion;
        const std::vector<cv::Point2f> points = {{3.0F, 4.0F}};
        const std::vector<std::uint8_t> labels = {7};
        const auto output = camera_fusion.update(points, labels);
        CHECK(output.first.size() == 1);
        CHECK(output.second.size() == 1);
        if (output.first.size() == 1 && output.second.size() == 1)
        {
            CHECK(output.second[0] == 7);
            CHECK(std::fabs(output.first[0].x - 3.0F) < 1e-6F);
            CHECK(std::fabs(output.first[0].y - 4.0F) < 1e-6F);
        }
    }

    // 255 哨兵必须被丢弃，不能变成融合身份。
    {
        fusion::CameraFusion camera_fusion;
        const std::vector<cv::Point2f> points = {{9.0F, 9.0F}};
        const std::vector<std::uint8_t> labels = {255};
        const auto output = camera_fusion.update(points, labels);
        CHECK(output.first.empty());
        CHECK(output.second.empty());
    }

    // 空检测也要能调用（用于推进相机丢失计时）。
    {
        fusion::CameraFusion camera_fusion;
        const auto output = camera_fusion.update({}, {});
        CHECK(output.first.empty());
    }

    // UDP 激光 + 相机：相机位置 5.2,5.1 应该被拉到激光的 5.0,5.0 附近。
    {
        fusion::CameraFusion camera_fusion;
        constexpr std::uint16_t port = 21411;
        std::string error;
        CHECK(camera_fusion.start_udp(port, &error));
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        CHECK(fd >= 0);
        if (fd >= 0)
        {
            sockaddr_in destination{};
            destination.sin_family = AF_INET;
            destination.sin_port = htons(port);
            ::inet_pton(AF_INET, "127.0.0.1", &destination.sin_addr);
            const char line[] = "L 1 5.0 5.0 0 0 0 0 0\n";
            ::sendto(fd, line, sizeof(line) - 1, 0,
                     reinterpret_cast<const sockaddr*>(&destination),
                     sizeof(destination));
            ::close(fd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        const std::vector<cv::Point2f> points = {{5.2F, 5.1F}};
        const std::vector<std::uint8_t> labels = {11};
        const auto output = camera_fusion.update(points, labels);
        CHECK(output.first.size() == 1);
        if (output.first.size() == 1)
            CHECK(std::fabs(output.first[0].x - 5.0F) < 0.2F);
        camera_fusion.stop();
    }

    if (failures == 0)
    {
        std::printf("camera fusion tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "camera fusion tests failed: %d\n", failures);
    return 1;
}
