#ifndef RM2027_PROTOCOL_H
#define RM2027_PROTOCOL_H

// Standalone 2027 referee protocol codec.  It intentionally has no serial or
// OpenCV dependency so it can be used by a PTY simulator on a development PC.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rm2027 {

constexpr uint8_t kSof = 0xA5;
constexpr std::size_t kHeaderSize = 5;
constexpr std::size_t kTailSize = 2;
constexpr std::size_t kMaxDataLength = 118; // 0x0301 maximum payload

enum : uint16_t {
    kCmdGameStatus = 0x0001,
    kCmdGameResult = 0x0002,
    kCmdRobotHp = 0x0003,
    kCmdRobotStatus = 0x0201,
    kCmdRadarMark = 0x020C,
    kCmdRadarInfo = 0x020E,
    kCmdRobotInteraction = 0x0301,
    kCmdRadarMap = 0x0305,
    kCmdRadioPosition = 0x0A01,
    kCmdRadioHp = 0x0A02,
    kCmdRadioAmmo = 0x0A03,
    kCmdRadioTeamStatus = 0x0A04,
    kCmdRadioBuffs = 0x0A05,
    kCmdRadioPassword = 0x0A06,
};

enum : uint16_t {
    kSubcmdRobotInteraction = 0x0200,
    kSubcmdRadarCommand = 0x0121,
    kServerId = 0x8080,
};

#pragma pack(push, 1)
struct FrameHeader {
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

struct RadarInfo {
    uint8_t vulnerability : 2;
    uint8_t vulnerating : 1;
    uint8_t encrypt_level : 2;
    uint8_t encrypt_changeable : 1;
    uint8_t reserved : 2;
    uint8_t armor_fortify_active : 1;
    uint8_t armor_fortify_remaining : 3;
    uint8_t armor_fortify_reserved : 4;
    uint16_t fortify_center_x;
    uint16_t fortify_center_y;
    uint16_t fortify_radius;
};

struct RadarCommand {
    uint8_t radar_cmd;
    uint8_t password_cmd;
    uint8_t password[6];
    uint8_t armor_fortify_cmd;
    uint16_t fortify_center_x;
    uint16_t fortify_center_y;
};

struct RobotInteractionHeader {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

struct RadioPositionData {
    uint16_t robot_1_x;
    uint16_t robot_1_y;
    uint16_t reserved_1_x;
    uint16_t reserved_1_y;
    uint16_t robot_3_x;
    uint16_t robot_3_y;
    uint16_t robot_4_x;
    uint16_t robot_4_y;
    uint16_t reserved_2_x;
    uint16_t reserved_2_y;
    uint16_t robot_7_x;
    uint16_t robot_7_y;
};

struct RadioHpData {
    uint16_t robot_1;
    uint16_t reserved_1;
    uint16_t robot_3;
    uint16_t robot_4;
    uint16_t reserved_2;
    uint16_t robot_7;
};

struct RadioAmmoData {
    uint16_t robot_1;
    uint16_t robot_3;
    uint16_t robot_4;
    uint16_t reserved;
    uint16_t robot_7;
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 5, "bad referee frame header size");
static_assert(sizeof(RadarInfo) == 8, "bad 0x020E size");
static_assert(sizeof(RadarCommand) == 13, "bad 0x0121 size");
static_assert(sizeof(RadioPositionData) == 24, "bad 0x0A01 size");
static_assert(sizeof(RadioHpData) == 12, "bad 0x0A02 size");
static_assert(sizeof(RadioAmmoData) == 10, "bad 0x0A03 size");

uint8_t crc8(const uint8_t* data, std::size_t size);
uint16_t crc16(const uint8_t* data, std::size_t size);

// Build a complete frame: header + cmd_id + payload + CRC16 (little endian).
std::vector<uint8_t> build_frame(uint16_t cmd_id, const uint8_t* payload,
                                  std::size_t payload_size, uint8_t seq);

inline std::vector<uint8_t> build_frame(uint16_t cmd_id,
                                        const std::vector<uint8_t>& payload,
                                        uint8_t seq) {
    return build_frame(cmd_id, payload.data(), payload.size(), seq);
}

// Parse and validate one complete frame.  `payload` is copied only after all
// CRC and length checks succeed.
bool parse_frame(const uint8_t* frame, std::size_t frame_size,
                 uint16_t& cmd_id, std::vector<uint8_t>& payload,
                 uint8_t* seq = nullptr, std::string* error = nullptr);

} // namespace rm2027

#endif
