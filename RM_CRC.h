#ifndef RM2027_CRC_H
#define RM2027_CRC_H

#include <cstddef>
#include <cstdint>

// RoboMaster referee protocol CRC implementation.
// The functions are inline so the project does not need a separate CRC .cpp.

inline uint8_t Get_CRC8_Check_Sum(const uint8_t* message, std::size_t length,
                                  uint8_t crc = 0xFF) {
    while (length--) {
        crc ^= *message++;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                                : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

inline bool Verify_CRC8_Check_Sum(const uint8_t* message, std::size_t length) {
    if (!message || length < 2)
        return false;
    return Get_CRC8_Check_Sum(message, length - 1) == message[length - 1];
}

inline void Append_CRC8_Check_Sum(uint8_t* message, std::size_t length) {
    if (!message || length < 2)
        return;
    message[length - 1] = Get_CRC8_Check_Sum(message, length - 1);
}

inline uint16_t Get_CRC16_Check_Sum(const uint8_t* message, std::size_t length,
                                    uint16_t crc = 0xFFFF) {
    while (length--) {
        crc ^= *message++;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                             : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

inline bool Verify_CRC16_Check_Sum(const uint8_t* message, std::size_t length) {
    if (!message || length < 3)
        return false;
    const uint16_t expected = Get_CRC16_Check_Sum(message, length - 2);
    const uint16_t actual = static_cast<uint16_t>(message[length - 2]) |
                            static_cast<uint16_t>(message[length - 1]) << 8;
    return expected == actual;
}

inline void Append_CRC16_Check_Sum(uint8_t* message, std::size_t length) {
    if (!message || length < 3)
        return;
    const uint16_t crc = Get_CRC16_Check_Sum(message, length - 2);
    message[length - 2] = static_cast<uint8_t>(crc & 0xFF);
    message[length - 1] = static_cast<uint8_t>(crc >> 8);
}

#endif
