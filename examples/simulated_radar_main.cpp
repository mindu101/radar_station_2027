// 这是一个最小的雷达主程序示例。
//
// 它不实现视觉算法，只负责模拟视觉算法产生的坐标，然后交给 messenger
// 通过裁判系统串口发送。把它连接到 tools/rm2027_referee_sim.py 创建的
// PTY 后，可以观察完整的收发流程。

#include "messenger.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) {
    stop_requested = 1;
}

} // namespace

namespace rm {

// messenger.cpp 需要这三个静态成员。真实项目一般会在自己的主程序或
// 配置文件中定义它们，这里给出一组可以直接运行的默认值。
std::vector<std::vector<uint8_t>> msg::robot_classes = {
    {1, 2, 3, 4, 5, 7},
    {101, 102, 103, 104, 105, 107},
};
std::chrono::milliseconds msg::interval{100};
bool msg::persistent_cache = false;

} // namespace rm

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "用法: %s /dev/pts/N [运行秒数]\n"
                     "例如: %s /dev/pts/3 60\n",
                     argv[0], argv[0]);
        return EXIT_FAILURE;
    }

    const char* referee_device = argv[1];
    const int run_seconds = argc >= 3 ? std::atoi(argv[2]) : 60;
    if (run_seconds <= 0) {
        std::fprintf(stderr, "运行秒数必须大于 0\n");
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    rm::msg radar;

    // 模拟视觉程序输出。坐标单位是米，机器人 ID 来自相机识别结果。
    // messenger 会把坐标转换成厘米，并按 0x0305 格式发送。
    radar.pos_from_camera = std::make_pair(
        std::vector<cv::Point2f>{
            cv::Point2f{5.0F, 3.0F},
            cv::Point2f{7.0F, 4.0F},
            cv::Point2f{9.0F, 6.0F},
            cv::Point2f{2.0F, 8.0F},
        },
        std::vector<uint8_t>{
            103, // 蓝方 3 号步兵
            104, // 蓝方 4 号步兵
            107, // 蓝方哨兵
            3,   // 红方 3 号步兵
        });

    // 打开裁判系统串口。对真实设备传 /dev/ttyUSB0，对 PTY 模拟器传
    // 它打印出来的 /dev/pts/N。
    if (!radar.run(referee_device)) {
        std::fprintf(stderr, "无法启动裁判系统通信: %s\n", referee_device);
        return EXIT_FAILURE;
    }

    std::printf("模拟雷达已启动，串口设备: %s\n", referee_device);
    std::printf("等待 0x0201 机器人身份帧后开始发送数据\n");

    const auto start = std::chrono::steady_clock::now();
    bool armor_request_1_sent = false;
    bool armor_request_2_sent = false;

    while (!stop_requested) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start);

        if (elapsed.count() >= run_seconds)
            break;

        // 模拟雷达第一次使用装甲加固机会。计数必须从 0 改为 1。
        if (!armor_request_1_sent && elapsed.count() >= 3) {
            const uint16_t center_x = 2500, center_y = 1200;
            radar.request_armor_fortify(1, center_x, center_y);
            armor_request_1_sent = true;
            std::printf("请求第一次装甲加固: center=(%hu,%hu) mm\n",
                        center_x, center_y);
        }

        // 模拟第二次使用。真实程序应当根据 0x020E 的剩余机会字段决定
        // 是否发送，这里只是为了演示计数值递增的格式。
        if (!armor_request_2_sent && elapsed.count() >= 10) {
            const uint16_t center_x = 3000, center_y = 1500;
            radar.request_armor_fortify(2, center_x, center_y);
            armor_request_2_sent = true;
            std::printf("请求第二次装甲加固: center=(%hu,%hu) mm\n",
                        center_x, center_y);
        }

        // 这里可以替换成真实视觉线程写入 pos_from_camera 的代码。
        // messenger 的发送线程会每 100 ms 读取一次当前坐标。
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    radar.stop();
    std::printf("模拟雷达已停止\n");
    return EXIT_SUCCESS;
}
