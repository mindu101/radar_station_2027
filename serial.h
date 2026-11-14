#ifndef RM2027_SERIAL_H
#define RM2027_SERIAL_H

#include "RM_CRC.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

namespace serial {

class stream {
protected:
    int fd_ = -1;

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
    explicit stream(int fd = -1) : fd_(fd) {}
    virtual ~stream() = default;

    int get_fd() const { return fd_; }

    ssize_t read(void* buffer, std::size_t length) {
        if (fd_ < 0)
            return -1;
        return ::read(fd_, buffer, length);
    }

    // Read the part after `offset`, then verify the complete CRC8-protected
    // buffer. The return value is the complete buffer length on success.
    ssize_t read_wCRC8(void* buffer, std::size_t length, std::size_t offset) {
        if (fd_ < 0 || offset > length)
            return -1;
        auto* bytes = static_cast<uint8_t*>(buffer);
        const ssize_t result = read_exact(fd_, bytes + offset, length - offset);
        if (result != static_cast<ssize_t>(length - offset))
            return result;
        return Verify_CRC8_Check_Sum(bytes, length)
                   ? static_cast<ssize_t>(length)
                   : static_cast<ssize_t>(-1);
    }

    // Read the part after `offset`, then verify the complete CRC16-protected
    // buffer. The return value is the complete buffer length on success.
    ssize_t read_wCRC16(void* buffer, std::size_t length, std::size_t offset) {
        if (fd_ < 0 || offset > length)
            return -1;
        auto* bytes = static_cast<uint8_t*>(buffer);
        const ssize_t result = read_exact(fd_, bytes + offset, length - offset);
        if (result != static_cast<ssize_t>(length - offset))
            return result;
        return Verify_CRC16_Check_Sum(bytes, length)
                   ? static_cast<ssize_t>(length)
                   : static_cast<ssize_t>(-1);
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
};

class terminal : public stream {
public:
    termios options{};

    explicit terminal(int fd = -1) : stream(fd) {
        if (fd_ >= 0)
            ::tcgetattr(fd_, &options);
    }

    int open(const char* filename = nullptr) {
        close();
        const char* device = filename ? filename : "/dev/ttyUSB0";
        fd_ = ::open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
            return -1;
        if (::tcgetattr(fd_, &options) < 0) {
            close();
            return -1;
        }
        return fd_;
    }

    int set_rate(speed_t rate) {
        return ::cfsetispeed(&options, rate) == 0 && ::cfsetospeed(&options, rate) == 0 ? 0 : -1;
    }

    int apply_options() {
        return fd_ >= 0 ? ::tcsetattr(fd_, TCSANOW, &options) : -1;
    }

    ssize_t write_wCRC16(void* buffer, std::size_t length) {
        if (fd_ < 0 || !buffer || length < 3)
            return -1;
        // referee_send owns this frame buffer, so appending the checksum in
        // place is safe and avoids an extra allocation on every packet.
        auto* bytes = static_cast<uint8_t*>(buffer);
        Append_CRC16_Check_Sum(bytes, length);
        return write_all(fd_, bytes, length);
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
                fd_ = candidate;
                result = 0;
                break;
            }
            ::close(candidate);
        }
        ::freeaddrinfo(addresses);
        return result;
    }

    ssize_t write(const void* buffer, std::size_t length) {
        if (fd_ < 0 || !buffer)
            return -1;
        return write_all(fd_, static_cast<const uint8_t*>(buffer), length);
    }
};

} // namespace serial

#endif
