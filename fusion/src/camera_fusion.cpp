#include "fusion/camera_fusion.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <sstream>

namespace fusion {

namespace {

// 上一届相机程序用 vector<uint8_t> 存 ids，未识别时 push_back(-1)，
// 实际值是 255。融合层里 255 不是合法类别，直接丢弃。
constexpr std::uint8_t kInvalidLabel = 255;

} // namespace

CameraFusion::~CameraFusion()
{
    stop();
}

bool CameraFusion::load_calibration(const std::string& path, std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fusion_.transform().load_json(path, error);
}

bool CameraFusion::load_config(const std::string& path, std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fusion_.config().load(path, error);
}

void CameraFusion::set_config(const FusionConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    fusion_.config() = config;
}

bool CameraFusion::start_udp(std::uint16_t port, std::string* error)
{
    if (port == 0)
        return true; // 纯相机模式
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_.load())
        return true;
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        if (error)
            *error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        if (error)
            *error = std::string("bind: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }
    fd_ = fd;
    running_ = true;
    thread_ = std::thread(&CameraFusion::receive_loop, this);
    return true;
}

void CameraFusion::stop()
{
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

void CameraFusion::handle_datagram(const char* buffer, std::size_t size, double now)
{
    std::vector<LidarTrack> lidar;
    std::vector<CameraTarget> camera;
    std::vector<RadioTarget> radio;
    std::istringstream lines(std::string(buffer, size));
    std::string line;
    while (std::getline(lines, line))
    {
        std::istringstream fields(line);
        char type = '\0';
        fields >> type;
        if (type == 'L')
        {
            LidarTrack track;
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
            CameraTarget target;
            if (fields >> target.label >> target.position.x >> target.position.y >>
                target.confidence)
                camera.push_back(target);
        }
        else if (type == 'R')
        {
            RadioTarget target;
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

    std::lock_guard<std::mutex> lock(mutex_);
    if (!lidar.empty())
        fusion_.update_lidar(lidar, now);
    if (!camera.empty())
        fusion_.update_camera(camera, now);
    if (!radio.empty())
        fusion_.update_radio(radio, now);
}

void CameraFusion::receive_loop()
{
    while (running_.load())
    {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(fd_, &read_set);
        timeval timeout{};
        timeout.tv_usec = 100000;
        const int ready = ::select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }
        if (ready == 0)
            continue;
        char buffer[65536];
        const ssize_t length = ::recv(fd_, buffer, sizeof(buffer) - 1, 0);
        if (length > 0)
        {
            buffer[length] = '\0';
            handle_datagram(buffer, static_cast<std::size_t>(length), now_seconds());
        }
    }
}

std::pair<std::vector<cv::Point2f>, std::vector<std::uint8_t>> CameraFusion::update(
    const std::vector<cv::Point2f>& points,
    const std::vector<std::uint8_t>& labels,
    const std::vector<float>& confidences)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<CameraTarget> detections;
    const std::size_t count = std::min(points.size(), labels.size());
    detections.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (labels[i] == kInvalidLabel)
            continue;
        if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y))
            continue;
        const float confidence = i < confidences.size() ? confidences[i] : 1.0F;
        detections.push_back(
            {static_cast<int>(labels[i]), {points[i].x, points[i].y}, confidence});
    }
    const double now = now_seconds();
    fusion_.update_camera(detections, now);
    const std::vector<FusedTarget> targets = fusion_.fuse(now);

    std::pair<std::vector<cv::Point2f>, std::vector<std::uint8_t>> output;
    output.first.reserve(targets.size());
    output.second.reserve(targets.size());
    for (const FusedTarget& target : targets)
    {
        if (target.label < 0 || target.label > 255)
            continue;
        output.first.emplace_back(static_cast<float>(target.position.x),
                                  static_cast<float>(target.position.y));
        output.second.push_back(static_cast<std::uint8_t>(target.label));
    }
    return output;
}

std::vector<FusedTarget> CameraFusion::fused()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fusion_.fuse(now_seconds());
}

} // namespace fusion
