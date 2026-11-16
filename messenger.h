#ifndef _MESSENGER_H_
#define _MESSENGER_H_

#include "serial.h"
#include <chrono>
#include <stdint.h>
#include <vector>
#include <thread>
#include "opencv2/core.hpp"
#define MAX_BUF 256
namespace rm
{
    //See RoboMaster file
    // 从这里到 #pragma pack(pop) 的所有 struct/union 都按 1 字节对齐，成员之间没有
    // 填充，所以 sizeof 就是各成员宽度之和（位域按编译器分配的单元计算）。
    #pragma pack(push, 1)
    // 裁判系统通用帧头，所有常规链路数据都以 0xA5 开始。
    // sizeof = 5 字节：SOF(1) + data_length(2) + seq(1) + CRC8(1)，pack(1) 下无填充。
    struct header
    {
        uint8_t SOF;
        uint16_t data_length;
        uint8_t seq;
        uint8_t CRC8;
    };
    // 0x0001 比赛状态，服务器每秒发送一次。
    // sizeof = 11 字节：1（game_type/game_progress 两个 4 位位域）+ 2 + 8。
    struct game_status
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    };
    //TODO 无
    // 0x0003 血量、基地血量和累计控制度，服务器每 3 Hz 发送。
    // sizeof = 24 字节：12 × uint16（struct 视图和 data[12] 都是 24）。
    union game_robot_HP
    {
        struct
        {
            uint16_t ally_1_robot_HP;
            uint16_t reserved;
            uint16_t ally_3_robot_HP;
            uint16_t ally_4_robot_HP;
            int16_t damage_difference;
            uint16_t ally_7_robot_HP;
            uint16_t outpost_R_HP;
            uint16_t ally_base_HP;
            uint16_t outpost_B_HP;
            uint16_t enemy_base_HP;
            uint16_t ally_control_degree;
            uint16_t enemy_control_degree;
        };
        uint16_t data[12];
    };
    //TODO 无

    // 0x0104 裁判警告，包含被处罚机器人和累计次数。
    // sizeof = 3 字节：3 × uint8。
    struct referee_warning
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    };
    // 0x0105 飞镖发射站状态。雷达当前只接收，不主动使用。
    // sizeof = 3 字节：remaining_time(1) + 一个 uint16_t 位域单元(2)。
    // 实测（GCC x86-64，小端）：byte1 = shot(bit0-2) | count(bit3-5) | target(bit6-7)，
    //   target 的最高位 bit8 落到 byte2 的 bit0；reserved 占 bit9-15（byte2 的 bit1-7）。
    // 注意：若改成 uint8_t 位域，target 会整个落到第 3 个字节，sizeof 变成 4 字节，
    //       两种写法的线上位布局不同，务必对照 2027 协议确认。

    struct dart_info
    {
        uint8_t dart_remaining_time;
        uint16_t dart_shot : 3;
        uint16_t dart_count : 3;
        uint16_t dart_target : 3;
        uint16_t reserved : 7;
    };
    // 0x0201 本机机器人状态，主控模块发送给对应机器人。
    // sizeof = 17 字节：robot_id(1) + level(1) + 5 × uint16(10)
    //                   + bullet_speed_limit(float, 4) + power_management 位域(1)。
    // 注意：常见官方 0x0201 robot_status_t 是 13 字节，bullet_speed_limit（float）
    //       一般属于 0x0202 射击状态。本文件和 tools/rm2027_referee_sim.py 都按
    //       17 字节收发，两处虽然自洽，但请对照 2027 协议 V1.0.0 核实 0x0201 布局。
    struct robot_status
    {
        uint8_t robot_id;
        //Unused below
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        float bullet_speed_limit;
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
    };
    //TODO 无
    // 2027 protocol, command 0x020C.  This is the mark progress for the
    // selected radar target, not the 2026 foe/ally bit masks.
    // 0x020C 雷达标记进度，范围通常为 0 到 150。
    // sizeof = 2 字节：单个 uint16_t mark_progress。
    // 注意：serial_info 里的 mark_progress 仍被注释掉，messenger.cpp 第 124 行
    //       赋值 serial_info.mark_progress 时编译失败（见下方 TODO）。
    //TODO
    //TODO 无
    //TODO 无
    //TODO 无
    //TODO 无
    //TODO 无
    //TODO 无
    //TODO 无
    //TODO 无

    struct radar_mark_data
    {
        uint16_t mark_progress;

    };
    //TODO 无
    // 0x020E 雷达自主决策状态，包括双倍易伤、密钥和装甲加固状态。
    // sizeof = 8 字节：2（两个 8 位位域字节）+ 3 × uint16。
    // 实测：byte0 = vulnerability:2 | vulnerating:1 | encrypt_level:2 |
    //                encrypt_changeable:1 | reserved:2；
    //       byte1 = armor_fortify_active:1 | remaining:3 | reserved:4。
    struct radar_info
    {
        uint8_t vulnerability : 2;       // 0..2 available double-vulnerability uses
        uint8_t vulnerating : 1;         // currently active
        uint8_t encrypt_level : 2;       // opponent interference level, 1..3
        uint8_t encrypt_changeable : 1;  // key may be changed
        uint8_t reserved : 2;
        uint8_t armor_fortify_active : 1;
        uint8_t armor_fortify_remaining : 3;
        uint8_t armor_fortify_reserved : 4;
        uint16_t fortify_center_x;
        uint16_t fortify_center_y;
        uint16_t fortify_radius;
    };
    // 机器人位置的六槽内部表示，坐标为 uint16_t 厘米。
    // 2027 协议中的保留槽位仍占用空间，因此保持 12 个坐标值。
    // sizeof = 24 字节：12 × uint16。
    union map_robot
    {
        struct
        {
            uint16_t hero_position_x;
            uint16_t hero_position_y;
            uint16_t engineer_position_x;
            uint16_t engineer_position_y;
            uint16_t infantry_3_position_x;
            uint16_t infantry_3_position_y;
            uint16_t infantry_4_position_x;
            uint16_t infantry_4_position_y;
            uint16_t infantry_5_position_x;
            uint16_t infantry_5_position_y;
            uint16_t sentry_position_x;
            uint16_t sentry_position_y;
        };
        uint16_t data[12];
    };
    // 0x0A01: positions sent by the official radio source.  The 2027 wire
    // layout contains robots 1, 3, 4 and 7 with two reserved pairs.
    // 0x0A01 雷达无线链路中的对方机器人坐标，单位为厘米。
    // sizeof = 24 字节：4 个机器人各 2 个坐标 + 2 组保留坐标，共 12 × uint16。
    struct radio_position_data
    {
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
    // 0x0A02 雷达无线链路中的对方机器人血量。
    // sizeof = 12 字节：6 × uint16。
    struct radio_hp_data
    {
        uint16_t robot_1;
        uint16_t reserved_1;
        uint16_t robot_3;
        uint16_t robot_4;
        uint16_t reserved_2;
        uint16_t robot_7;
    };
    // 0x0A03 雷达无线链路中的对方机器人允许发弹量。
    // sizeof = 10 字节：5 × uint16。
    struct radio_ammo_data
    {
        uint16_t robot_1;
        uint16_t robot_3;
        uint16_t robot_4;
        uint16_t reserved;
        uint16_t robot_7;
    };
    // 0x0A04 对方队伍经济、增益点和地形控制状态。
    // sizeof = 12 字节：uint16 + uint16 + uint32 + uint32。
    struct radio_team_status
    {
        uint16_t remaining_coins;
        uint16_t total_coins;
        uint32_t active_buffs;
        uint32_t terrain_and_control;
    };
    // 0x0A05 对方机器人增益和主要状态。暂按协议保存为原始 47 字节。
    // sizeof = 47 字节：原始字节数组，未做字段解析。
    struct radio_buff_data
    {
        uint8_t data[47];
    };
    // 0x0301 的子命令 0x0121，雷达发送的 13 字节自主决策数据。
    // sizeof = 13 字节：radar_cmd(1) + password_cmd(1) + password(6)
    //                   + armor_fortify_cmd(1) + 2 × uint16(4)。
    struct radar_command_data
    {
        uint8_t radar_cmd;
        uint8_t password_cmd;
        uint8_t password[6];
        uint8_t armor_fortify_cmd;
        uint16_t fortify_center_x;
        uint16_t fortify_center_y;
    };
    // 0x0305 小地图位置数据，前 24 字节为对方，后 24 字节为己方。
    // sizeof = 48 字节：opponent(24) + ally(24)，与 data[24] 一致。
    union both_map_robot
    {
        struct
        {
            union map_robot opponent;
            union map_robot ally;
        };
        uint16_t data[24];
    };
    //Communicate with radio
    // 发给外部雷达无线模块的短帧，供上一届 TCP 无线链路接口使用。
    // sizeof = 3 字节：header + robot_id + encrypt_level，各 1 字节。
    struct rf_tx_frame
    {
        uint8_t header; // 0xA5
        uint8_t robot_id;
        uint8_t encrypt_level;
    };
    //Communicate with sentry
    // 雷达或视觉使用的机器人浮点坐标，单位为米。
    // sizeof = 48 字节：12 × float。
    union robot_pos
    {
        struct
        {
            float hero_x;
            float hero_y;
            float engineer_x;
            float engineer_y;
            float standard_3_x;
            float standard_3_y;
            float standard_4_x;
            float standard_4_y;
            float standard_5_x;
            float standard_5_y;
            float sentry_x;
            float sentry_y;
        };
        float data[12];
    };
    // 对方和己方两组机器人浮点坐标。
    // sizeof = 96 字节：opponent(48) + ally(48)，与 data[24] 一致。
    union both_robot_pos
    {
        struct
        {
            union robot_pos opponent;
            union robot_pos ally;
        };
        float data[24];
    };
    // 0x0301 机器人交互数据的统一数据头和内容。
    // sizeof = 54 字节：6（三个 uint16 头）+ 48（联合体中较大的 robot_pos）。
    // 注意：54 只是 C++ 对象大小；线上实际长度由帧头 data_length 决定：
    //       0x0121 子命令为 6 + 13 = 19 字节，机器人坐标子命令为 6 + 48 = 54 字节。
    struct robot_interaction_data
    {
        struct
        {
            uint16_t data_cmd_id;
            uint16_t sender_id;
            uint16_t receiver_id;
        } header;
        union
        {
            struct radar_command_data cmd;
            union robot_pos robot_pos;
        };
    };
    // 以下断言在 GCC x86-64 上编译通过。未断言的结构体大小见各自注释。
    static_assert(sizeof(radar_mark_data)==2,"0x020C must be 2 bytes");
    static_assert(sizeof(game_status)==11,"0x0001 must be 11 bytes");
    static_assert(sizeof(game_robot_HP)==24,"0x0003 must be 24 bytes");
    static_assert(sizeof(radar_info)==8,"0x020E must be 8 bytes");
    static_assert(sizeof(radio_position_data)==24,"0x0A01 must be 24 bytes");
    static_assert(sizeof(radio_hp_data)==12,"0x0A02 must be 12 bytes");
    static_assert(sizeof(radio_ammo_data)==10,"0x0A03 must be 10 bytes");
    static_assert(sizeof(radio_team_status)==12,"0x0A04 must be 12 bytes");
    static_assert(sizeof(radio_buff_data)==47,"0x0A05 must be 47 bytes");
    static_assert(sizeof(radar_command_data)==13,"0x0121 must be 13 bytes");
    // 接收和发送时使用的最大协议帧缓存。具体内容由 cmd_id 决定。
    // sizeof = 256 字节（MAX_BUF）：struct 视图为 5(header) + 2(cmd_id)
    //   + 54(最大载荷 robot_interaction_data) = 61 字节，与 data[256] 取并集后为 256。
    // 完整线上帧长度 = 5(header) + 2(cmd_id) + data_length + 2(CRC16 帧尾)。
    union frame
    {
        struct
        {
            struct header header;
            uint16_t cmd_id;
            union
            {
                struct game_status game_status;
                uint8_t winner;
                union game_robot_HP game_robot_HP;
                struct referee_warning referee_warning;
                struct dart_info dart_info;
                struct robot_status robot_status;
                struct radar_mark_data radar_mark_data;
                struct radar_info radar_info;
                struct radio_position_data radio_position_data;
                struct radio_hp_data radio_hp_data;
                struct radio_ammo_data radio_ammo_data;
                struct radio_team_status radio_team_status;
                struct radio_buff_data radio_buff_data;
                struct robot_interaction_data robot_interaction_data;
                union map_robot map_robot;
                uint8_t password[6];
            };
        };
        uint8_t data[MAX_BUF];
    };
    // 帧尾 CRC16 校验，sizeof = 2 字节。
    typedef uint16_t frame_tail;
    #pragma pack(pop)
    // 供雷达业务逻辑读取的已解析状态，不是裁判系统的原始数据帧。
    // 注意：本结构体在 #pragma pack(pop) 之后，按默认对齐（对齐要求 2 字节）。
    // sizeof = 18 字节：6（3 × uint16）+ 1（robot_id）+ 1（game_progress 位域）
    //   + 1（foe_mark）+ 1（ally_mark）+ 2（其余位域）+ 6（3 × uint16）。
    //   实测 fortify_center_x 在偏移 12，fortify_radius 在偏移 16，结束于 18。
    struct serial_info
    {
        uint16_t stage_remain_time;
        uint16_t ally_outpost_base_HP;
        uint16_t foe_outpost_base_HP;
        uint8_t robot_id;
        uint8_t game_progress:3;
        // uint16_t mark_progress;
        //TODO
        // Kept for source compatibility with the 2026 consumer code.  They
        // are always zero in 2027; use mark_progress instead.
        uint8_t foe_mark;
        uint8_t ally_mark;
        uint8_t vulnerability:2;
        bool vulnerating:1;
        uint8_t encrypt_level:2;
        bool encrypt_changeable:1;
        bool armor_fortify_active:1;
        uint8_t armor_fortify_remaining:3;
        uint16_t fortify_center_x;
        uint16_t fortify_center_y;
        uint16_t fortify_radius;
    };
    // 雷达通信管理类，负责串口/TCP 接收线程、周期发送线程和状态缓存。
    class msg
    {
        union frame recv_buf;
        union frame rf_buf;
        std::chrono::steady_clock::time_point timestamp[3];
        volatile uint8_t cmd;
        uint8_t seq;
        bool running;
        std::thread rx,tx,rf;
        void referee_callback(union frame& buf);
        void poll(void);
        void receive(serial::stream& src, union frame& buf, bool(*reopen)(serial::stream&));
    public:
        serial::terminal referee;
        serial::tcp radio;
        using timer=std::chrono::steady_clock;
        static std::vector<std::vector<uint8_t>> robot_classes; //HERO=0,ENGINEER,INFANTRY3,INFANTRY4,INFANTRY5,SENTRY
        static std::chrono::milliseconds interval;
        static bool persistent_cache; //Send robot's last pos if lost, and guess drone's pos, instead of ignoring
        volatile struct serial_info serial_info;
        std::pair<std::vector<cv::Point2f>, std::vector<uint8_t>> pos_from_camera;
        volatile union both_robot_pos pos_from_sentry;
        volatile union map_robot pos_from_radio;
        volatile struct radio_hp_data hp_from_radio;
        volatile struct radio_ammo_data ammo_from_radio;
        volatile struct radio_team_status team_status_from_radio;
        volatile struct radio_buff_data buffs_from_radio;
        volatile int8_t pos_cache[12]; //Cache control, positive for sentry, negative for radio and zero for video
        uint8_t guessed_password[6];
        volatile uint8_t password_retries;
        volatile uint8_t vulnerated; //Double vulnerability trigger
        // Commands controlled by the radar application.  Each counter is
        // monotonic; the referee ignores a repeated value as required by
        // protocol 0x0121.
        volatile uint8_t armor_fortify;
        volatile uint16_t fortify_center_x;
        volatile uint16_t fortify_center_y;
        msg(int referee_fd=-1, int radio_fd=-1):recv_buf{},rf_buf{},timestamp{timer::now(),timer::now(),timer::now()},cmd(0),seq(0),running(false),referee(referee_fd),radio(radio_fd),serial_info{},pos_from_camera{},pos_from_sentry{},pos_from_radio{},hp_from_radio{},ammo_from_radio{},team_status_from_radio{},buffs_from_radio{},pos_cache{0},guessed_password{0},password_retries(0),vulnerated(0),armor_fortify(0),fortify_center_x(0),fortify_center_y(0){}
        msg(const msg&)=delete; //Prevents thread problems
        msg& operator=(const msg&)=delete;
        msg(msg&&)=default;
        msg& operator=(msg&&)=default;
        static bool serial_quick_setup(serial::terminal& port, const char* filename=0);
        static std::vector<uint8_t> generate_password(size_t len);
        bool referee_send(uint16_t cmd_id, uint16_t data_length, const void* data);
        bool run(const char* referee_tty=0, const char* radio_host=0, uint16_t radio_port=0);
        void stop(void);
        ~msg();
    };
}

#endif
