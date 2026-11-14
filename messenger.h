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
    struct header
    {
        uint8_t SOF;
        uint16_t data_length;
        uint8_t seq;
        uint8_t CRC8;
    };
    struct game_status
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    };
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
    struct referee_warning
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    };
    struct dart_info
    {
        uint8_t dart_remaining_time;
        uint8_t dart_shot : 3;
        uint8_t dart_count : 3;
        uint8_t dart_target : 3;
        uint8_t reserved;
    };
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
    struct radar_mark_data
    {
        uint16_t mark_progress;
    };
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
    struct radio_hp_data
    {
        uint16_t robot_1;
        uint16_t reserved_1;
        uint16_t robot_3;
        uint16_t robot_4;
        uint16_t reserved_2;
        uint16_t robot_7;
    };
    struct radio_ammo_data
    {
        uint16_t robot_1;
        uint16_t robot_3;
        uint16_t robot_4;
        uint16_t reserved;
        uint16_t robot_7;
    };
    struct radio_team_status
    {
        uint16_t remaining_coins;
        uint16_t total_coins;
        uint32_t active_buffs;
        uint32_t terrain_and_control;
    };
    struct radio_buff_data
    {
        uint8_t data[47];
    };
    struct radar_command_data
    {
        uint8_t radar_cmd;
        uint8_t password_cmd;
        uint8_t password[6];
        uint8_t armor_fortify_cmd;
        uint16_t fortify_center_x;
        uint16_t fortify_center_y;
    };
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
    struct rf_tx_frame
    {
        uint8_t header; // 0xA5
        uint8_t robot_id;
        uint8_t encrypt_level;
    };
    //Communicate with sentry
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
    union both_robot_pos
    {
        struct
        {
            union robot_pos opponent;
            union robot_pos ally;
        };
        float data[24];
    };
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
    struct serial_info //See RoboMaster doc
    {
        uint16_t stage_remain_time;
        uint16_t ally_outpost_base_HP;
        uint16_t foe_outpost_base_HP;
        uint8_t robot_id;
        uint8_t game_progress:3;
        uint16_t mark_progress;
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
