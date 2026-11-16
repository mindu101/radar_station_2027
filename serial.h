#ifndef RM2027_SERIAL_H
#define RM2027_SERIAL_H

#include "RM_CRC.h"

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
//封装一下linux的c语言底层调用，变成容易理解的类，集成CRC8.CRC16的裁判
namespace serial {

class stream {
protected:
    // 读写线程和 stop() 会并发访问 fd，用原子量避免 close 与 read 之间的数据竞争。
    std::atomic<int> fd_{-1};

    static ssize_t read_exact(int fd, uint8_t* buffer, std::size_t length) {
        std::size_t received = 0;
        while (received < length) {
            const ssize_t result = ::read(fd, buffer + received, length - received);
            if (result > 0) {
                received += static_cast<std::size_t>(result);
                continue;
            }
            if (result < 0 && errno == EINTR)
                continue;
            return result;
        }
        return static_cast<ssize_t>(received);
    }

    static ssize_t write_all(int fd, const uint8_t* buffer, std::size_t length) {
        std::size_t sent = 0;
        while (sent < length) {
            const ssize_t result = ::write(fd, buffer + sent, length - sent);
            if (result > 0) {
                sent += static_cast<std::size_t>(result);
                continue;
            }
            if (result < 0 && errno == EINTR)
                continue;
            return result;
        }
        return static_cast<ssize_t>(sent);
    }

public:
    // CRC 校验失败（数据已被读走，调用方应重新同步帧头，而不是重连）。
    // 用 -2 与 ::read 的错误返回值 -1、EOF/超时的 0 区分开。
    static constexpr ssize_t kChecksumMismatch = -2;

    explicit stream(int fd = -1) : fd_(fd) {}
    virtual ~stream() = default;

    int get_fd() const { return fd_.load(); }

    ssize_t read(void* buffer, std::size_t length) {
        const int fd = fd_.load();
        if (fd < 0)
            return -1;
        return ::read(fd, buffer, length);
    }

    // Read the part after `offset`, then verify the complete CRC8-protected
    // buffer. The return value is the complete buffer length on success.
    ssize_t read_wCRC8(void* buffer, std::size_t length, std::size_t offset) {
        const int fd = fd_.load();
        if (fd < 0 || offset > length)
            return -1;
        auto* bytes = static_cast<uint8_t*>(buffer);
        const ssize_t result = read_exact(fd, bytes + offset, length - offset);
        if (result != static_cast<ssize_t>(length - offset))
            return result;
        return Verify_CRC8_Check_Sum(bytes, length)
                   ? static_cast<ssize_t>(length)
                   : kChecksumMismatch;
    }

    // Read the part after `offset`, then verify the complete CRC16-protected
    // buffer. The return value is the complete buffer length on success.
    ssize_t read_wCRC16(void* buffer, std::size_t length, std::size_t offset) {
        const int fd = fd_.load();
        if (fd < 0 || offset > length)
            return -1;
        auto* bytes = static_cast<uint8_t*>(buffer);
        const ssize_t result = read_exact(fd, bytes + offset, length - offset);
        if (result != static_cast<ssize_t>(length - offset))
            return result;
        return Verify_CRC16_Check_Sum(bytes, length)
                   ? static_cast<ssize_t>(length)
                   : kChecksumMismatch;
    }

    void close() {
        // exchange 保证同一个 fd 只被 close 一次。
        const int fd = fd_.exchange(-1);
        if (fd >= 0)
            ::close(fd);
    }
};

class terminal : public stream {
public:
    termios options{};

    explicit terminal(int fd = -1) : stream(fd) {
        if (fd >= 0)
            ::tcgetattr(fd, &options);
    }

    int open(const char* filename = nullptr) {
        close();
        const char* device = filename ? filename : "/dev/ttyUSB0";
        const int fd = ::open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
            return -1;
        fd_.store(fd);
        if (::tcgetattr(fd, &options) < 0) {
            close();
            return -1;
        }
        return fd;
    }

    int set_rate(speed_t rate) {
        return ::cfsetispeed(&options, rate) == 0 && ::cfsetospeed(&options, rate) == 0 ? 0 : -1;
    }

    int apply_options() {
        const int fd = fd_.load();
        return fd >= 0 ? ::tcsetattr(fd, TCSANOW, &options) : -1;
    }

    ssize_t write_wCRC16(void* buffer, std::size_t length) {
        const int fd = fd_.load();
        if (fd < 0 || !buffer || length < 3)
            return -1;
        // referee_send owns this frame buffer, so appending the checksum in
        // place is safe and avoids an extra allocation on every packet.
        auto* bytes = static_cast<uint8_t*>(buffer);
        Append_CRC16_Check_Sum(bytes, length);
        return write_all(fd, bytes, length);
    }
};

class tcp : public stream {
public:
    explicit tcp(int fd = -1) : stream(fd) {}

    int connect(const char* host, uint16_t port) {
        close();
        if (!host)
            return -1;

        const std::string service = std::to_string(port);
        addrinfo hints{};
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_family = AF_UNSPEC;
        addrinfo* addresses = nullptr;
        if (::getaddrinfo(host, service.c_str(), &hints, &addresses) != 0)
            return -1;

        int result = -1;
        for (addrinfo* address = addresses; address; address = address->ai_next) {
            const int candidate = ::socket(address->ai_family, address->ai_socktype,
                                           address->ai_protocol);
            if (candidate < 0)
                continue;
            if (::connect(candidate, address->ai_addr, address->ai_addrlen) == 0) {
                fd_.store(candidate);
                result = 0;
                break;
            }
            ::close(candidate);
        }
        ::freeaddrinfo(addresses);
        return result;
    }

    ssize_t write(const void* buffer, std::size_t length) {
        const int fd = fd_.load();
        if (fd < 0 || !buffer)
            return -1;
        return write_all(fd, static_cast<const uint8_t*>(buffer), length);
    }
};

} // namespace serial

#endif
