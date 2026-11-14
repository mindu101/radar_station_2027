#include "rm2027_protocol.h"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

int main() {
    const uint8_t header_without_crc[] = {0xA5, 0x08, 0x00, 0x03};
    assert(rm2027::crc8(header_without_crc, sizeof(header_without_crc)) == 0xC2);

    std::vector<uint8_t> payload(sizeof(rm2027::RadarCommand));
    for (std::size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>(i);

    const auto frame = rm2027::build_frame(rm2027::kCmdRobotInteraction, payload, 7);
    assert(frame.size() == 5 + 2 + payload.size() + 2);

    uint16_t cmd = 0;
    uint8_t seq = 0;
    std::vector<uint8_t> decoded;
    std::string error;
    assert(rm2027::parse_frame(frame.data(), frame.size(), cmd, decoded, &seq, &error));
    assert(cmd == rm2027::kCmdRobotInteraction);
    assert(seq == 7);
    assert(decoded == payload);

    auto bad_crc = frame;
    bad_crc[5] ^= 1;
    assert(!rm2027::parse_frame(bad_crc.data(), bad_crc.size(), cmd, decoded, &seq, &error));
    std::cout << "rm2027 protocol tests passed\n";
}
