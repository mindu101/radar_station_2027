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
    #pragma pack(push, 1)
    // 裁判系统通用帧头，所有常规链路数据都以 0xA5 开始。
    struct header
    {
        uint8_t SOF;
        uint16_t data_length;
        uint8_t seq;
        uint8_t CRC8;
    };
    // 0x0001 比赛状态，服务器每秒发送一次。
    struct game_status
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    };
    // 0x0003 血量、基地血量和累计控制度，服务器每 3 Hz 发送。
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
    // 0x0104 裁判警告，包含被处罚机器人和累计次数。
    struct referee_warning
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    };
    // 0x0105 飞镖发射站状态。雷达当前只接收，不主动使用。
    struct dart_info
    {
        uint8_t dart_remaining_time;
        uint8_t dart_shot : 3;
        uint8_t dart_count : 3;
        uint8_t dart_target : 3;
        uint8_t reserved;
    };
    // 0x0201 本机机器人状态，主控模块发送给对应机器人。
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
    // 2027 protocol, command 0x020C.  This is the mark progress for the
    // selected radar target, not the 2026 foe/ally bit masks.
    // 0x020C 雷达标记进度，范围通常为 0 到 150。
    //TODO
    struct radar_mark_data
    {
        // uint16_t mark_progress;
        uint16_t foe_mark:6;
        uint16_t ally_mark:6;
        uint16_t reserved:4;

    };
    // 0x020E 雷达自主决策状态，包括双倍易伤、密钥和装甲加固状态。
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
    struct radio_ammo_data
    {
        uint16_t robot_1;
        uint16_t robot_3;
        uint16_t robot_4;
        uint16_t reserved;
        uint16_t robot_7;
    };
    // 0x0A04 对方队伍经济、增益点和地形控制状态。
    struct radio_team_status
    {
        uint16_t remaining_coins;
        uint16_t total_coins;
        uint32_t active_buffs;
        uint32_t terrain_and_control;
    };
    // 0x0A05 对方机器人增益和主要状态。暂按协议保存为原始 47 字节。
    struct radio_buff_data
    {
        uint8_t data[47];
    };
    // 0x0301 的子命令 0x0121，雷达发送的 13 字节自主决策数据。
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
    struct rf_tx_frame
    {
        uint8_t header; // 0xA5
        uint8_t robot_id;
        uint8_t encrypt_level;
    };
    //Communicate with sentry
    // 雷达或视觉使用的机器人浮点坐标，单位为米。
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
    // static_assert(sizeof(radar_mark_data)==2,"0x020C must be 2 bytes");
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
    typedef uint16_t frame_tail;
    #pragma pack(pop)
    // 供雷达业务逻辑读取的已解析状态，不是裁判系统的原始数据帧。
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
