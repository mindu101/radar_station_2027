#include "rm2027_protocol.h"

#include <cstring>

namespace rm2027 {

uint8_t crc8(const uint8_t* data, std::size_t size) {
    uint8_t crc = 0xFF;
    while (size--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                                : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

uint16_t crc16(const uint8_t* data, std::size_t size) {
    uint16_t crc = 0xFFFF;
    while (size--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                             : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

std::vector<uint8_t> build_frame(uint16_t cmd_id, const uint8_t* payload,
                                 std::size_t payload_size, uint8_t seq) {
    if (payload_size > kMaxDataLength)
        return {};
    const std::size_t total = kHeaderSize + sizeof(cmd_id) + payload_size + kTailSize;
    std::vector<uint8_t> out(total, 0);
    out[0] = kSof;
    const uint16_t wire_length = static_cast<uint16_t>(payload_size);
    std::memcpy(out.data() + 1, &wire_length, sizeof(wire_length));
    out[3] = seq;
    out[4] = crc8(out.data(), 4);
    std::memcpy(out.data() + kHeaderSize, &cmd_id, sizeof(cmd_id));
    if (payload_size && payload)
        std::memcpy(out.data() + kHeaderSize + sizeof(cmd_id), payload, payload_size);
    const uint16_t checksum = crc16(out.data(), total - kTailSize);
    std::memcpy(out.data() + total - kTailSize, &checksum, sizeof(checksum));
    return out;
}

bool parse_frame(const uint8_t* frame, std::size_t frame_size,
                 uint16_t& cmd_id, std::vector<uint8_t>& payload,
                 uint8_t* seq, std::string* error) {
    auto fail = [&](const char* why) {
        if (error) *error = why;
        return false;
    };
    payload.clear();
    if (!frame || frame_size < kHeaderSize + sizeof(cmd_id) + kTailSize)
        return fail("frame too short");
    if (frame[0] != kSof)
        return fail("bad SOF");
    if (crc8(frame, 4) != frame[4])
        return fail("bad header CRC8");
    uint16_t data_length = 0;
    std::memcpy(&data_length, frame + 1, sizeof(data_length));
    if (data_length > kMaxDataLength)
        return fail("data length exceeds protocol maximum");
    const std::size_t expected = kHeaderSize + sizeof(cmd_id) + data_length + kTailSize;
    if (frame_size != expected)
        return fail("frame length does not match data_length");
    uint16_t got_crc = 0;
    std::memcpy(&got_crc, frame + expected - kTailSize, sizeof(got_crc));
    if (crc16(frame, expected - kTailSize) != got_crc)
        return fail("bad frame CRC16");
    std::memcpy(&cmd_id, frame + kHeaderSize, sizeof(cmd_id));
    payload.assign(frame + kHeaderSize + sizeof(cmd_id),
                   frame + kHeaderSize + sizeof(cmd_id) + data_length);
    if (seq) *seq = frame[3];
    if (error) error->clear();
    return true;
}

} // namespace rm2027
