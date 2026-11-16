#include "messenger.h"
#include "serial.h"
#include "RM_CRC.h"
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#include <poll.h>
#define SERIAL_SPEED B115200
namespace rm
{
    namespace
    {
        // 当前帧的 payload 是否至少有 required 字节。不够就整帧丢弃，
        // 避免用到接收缓冲区里上一帧残留的字节。
        bool payload_long_enough(const union frame& buf, std::size_t required)
        {
            if(buf.header.data_length>=required)
                return true;
            #ifndef NDEBUG
            fprintf(stderr,"Short payload for cmd 0x%04hx: %hu < %zu\n",
                buf.cmd_id,buf.header.data_length,required);
            #endif
            return false;
        }
    }
    //RODO
    //密码不需要盲猜了
    std::vector<uint8_t> msg::generate_password(size_t len)
    {
        static const uint8_t hard_bytes[] = {
            0x5A, 0x55, 0x33, 0x30, 0x6F
        };
        static std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
        static std::uniform_int_distribution<size_t> dist(0, std::size(hard_bytes) - 1);
        std::vector<uint8_t> password;
        password.reserve(len);
        for(;len>0;--len)
            password.push_back(hard_bytes[dist(rng)]);
        return password;
    }
    bool msg::serial_quick_setup(serial::terminal& port, const char* filename)
    {
        // fd 已经打开（构造时传入，或重连前刚 close）时只重新配置 termios，
        // 不盲目 reopen；fd 无效时才按路径打开。
        if(port.get_fd()<0)
        {
            if(filename)
            {
                if(port.open(filename)<0)
                    return false;
            }
            else if(port.open()<0)
                return false;
        }
        else if(::tcgetattr(port.get_fd(),&port.options)<0)
            return false;
        port.set_rate(SERIAL_SPEED);
        port.options.c_cflag&=~(CSIZE|CSTOPB|PARENB);
        port.options.c_cflag|=(CS8|CREAD|CLOCAL);
        port.options.c_iflag&=~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
        port.options.c_oflag&=~OPOST;
        port.options.c_lflag&=~(ICANON|ECHO|ISIG);
        port.options.c_cc[VMIN] = 32;
        port.options.c_cc[VTIME] = 2;
        return port.apply_options()==0;
    }
    void msg::referee_callback(union frame& buf)
    {
        switch(buf.cmd_id)
        {
            case 0x0001:
                if(!payload_long_enough(buf,sizeof(buf.game_status)))
                    break;
                {
                    uint8_t previous_progress;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        previous_progress=serial_info.game_progress;
                    }
                    if(buf.game_status.game_progress!=previous_progress)
                        printf("Game state changed to: %hhu\n",buf.game_status.game_progress);
                }
                printf("Timer: %hu:%hu\n",buf.game_status.stage_remain_time/60,buf.game_status.stage_remain_time%60);
                if(buf.game_status.game_progress==4)
                {
                    const uint8_t triggered=vulnerated.load();
                    if((buf.game_status.stage_remain_time<300&&triggered<1)||(buf.game_status.stage_remain_time<180&&triggered<2))
                    {
                        ++vulnerated;
                        printf("Double vulnerabilitiy triggered due to timeout\n");
                    }
                }
                else
                {
                    vulnerated=0;
                    cmd=0;
                    radar_cmd_sent=0;
                    password_update_sent=false;
                }
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    serial_info.game_progress=buf.game_status.game_progress;
                    if(buf.game_status.game_progress!=5)
                        serial_info.stage_remain_time=buf.game_status.stage_remain_time;
                }
            break;
            case 0x0002:
                if(!payload_long_enough(buf,sizeof(buf.winner)))
                    break;
                switch(buf.winner)
                {
                    case 0:
                        printf("Result: Draw (or unexpectedly ended)!\n");
                        break;
                    case 1:
                        printf("Result: Red wins!\n");
                        break;
                    case 2:
                        printf("Result: Blue wins!\n");
                        break;
                }
            break;
            case 0x0003:
                if(!payload_long_enough(buf,sizeof(buf.game_robot_HP)))
                    break;
                {
                    uint8_t own_id;
                    uint8_t progress;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        own_id=serial_info.robot_id;
                        progress=serial_info.game_progress;
                    }
                    // 2027 协议 0x0003 固定 24 字节（表 1-8）：outpost_R/outpost_B
                    // 是绝对的红/蓝前哨，ally_base/enemy_base 已相对己方，所以
                    // 只需按 robot_id 百位选出己方的前哨。
                    const bool blue=own_id/100;
                    const uint16_t ally_outpost=blue?buf.game_robot_HP.outpost_B_HP:buf.game_robot_HP.outpost_R_HP;
                    const uint16_t foe_outpost=blue?buf.game_robot_HP.outpost_R_HP:buf.game_robot_HP.outpost_B_HP;
                    const uint16_t ally_base=buf.game_robot_HP.ally_base_HP;
                    const uint16_t foe_base=buf.game_robot_HP.enemy_base_HP;
                    printf("HP from serial: ally1=%hu ally3=%hu ally4=%hu ally7=%hu allyOutpost=%hu allyBase=%hu foeOutpost=%hu foeBase=%hu damageDiff=%hd\n",
                        buf.game_robot_HP.ally_1_robot_HP,buf.game_robot_HP.ally_3_robot_HP,
                        buf.game_robot_HP.ally_4_robot_HP,buf.game_robot_HP.ally_7_robot_HP,
                        ally_outpost,ally_base,foe_outpost,foe_base,
                        buf.game_robot_HP.damage_difference);
                    const uint8_t triggered=vulnerated.load();
                    if(progress==4&&((ally_outpost<1000&&triggered<1)||(ally_base<5000&&triggered<2)))
                    {
                        ++vulnerated;
                        printf("Double vulnerabilitiy triggered due to low HP\n");
                    }
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        serial_info.ally_outpost_base_HP=ally_outpost+ally_base;
                        serial_info.foe_outpost_base_HP=foe_outpost+foe_base;
                    }
                }
            break;
            case 0x0104:
                if(!payload_long_enough(buf,sizeof(buf.referee_warning)))
                    break;
                printf("Robot %hhu is %s %hhu time(s)!\n",buf.referee_warning.offending_robot_id,buf.referee_warning.level>=3?"punished":"warned",buf.referee_warning.count);
            break;
            case 0x0105:
                if(!payload_long_enough(buf,sizeof(buf.dart_info)))
                    break;
            //用飞镖的信息作为按键触发双倍易伤，很聪明
                if(buf.dart_info.dart_target!=pre_dart_target.load())
                {
                    pre_dart_target=buf.dart_info.dart_target;
                    printf("Dart target switched\n");
                    uint8_t progress;
                    uint8_t available;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        progress=serial_info.game_progress;
                        available=serial_info.vulnerability;
                    }
                    if(progress==4&&available>0)
                    {
                        ++vulnerated;
                        printf("Double vulnerabilitiy triggered manually\n");
                    }
                }
            break;
            case 0x0201:
                //red 9 ，blue 109
                // 这里只用 robot_id，不要求完整的 robot_status 长度。
                if(!payload_long_enough(buf,sizeof(buf.robot_status.robot_id)))
                    break;
                printf("Robot ID: %hhu (%s)\n",buf.robot_status.robot_id,buf.robot_status.robot_id/100?"Blue":"Red");
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    serial_info.robot_id=buf.robot_status.robot_id;
                }
            break;
            case 0x020C:
                if(!payload_long_enough(buf,sizeof(buf.radar_mark_data)))
                    break;
                {
                    const uint16_t mark=buf.radar_mark_data.mark_progress;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        serial_info.mark_progress=mark;
                    }
                    #ifndef NDEBUG
                    printf("Radar mark progress: %hu\n",mark);
                    #endif
                }
            break;
            case 0x020E:
                if(!payload_long_enough(buf,sizeof(buf.radar_info)))
                    break;
                printf("Double vulnerabilities: %hhu%c\n",buf.radar_info.vulnerability,buf.radar_info.vulnerating?'*':' ');
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    // cmd 记录累计获得的双倍易伤次数，必须按增量累加：如果开局
                    // 直接是 2 次可用，每次只 +1 会让 cmd 永远到不了 2，第二次请求发不出去。
                    if(serial_info.vulnerability<buf.radar_info.vulnerability)
                        cmd+=buf.radar_info.vulnerability-serial_info.vulnerability;
                    serial_info.vulnerability=buf.radar_info.vulnerability;
                    serial_info.vulnerating=buf.radar_info.vulnerating;
                }
                printf("Encrypt level: %hhu%c\n",buf.radar_info.encrypt_level,buf.radar_info.encrypt_changeable?'*':' ');
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    if(serial_info.encrypt_level<buf.radar_info.encrypt_level)
                    {
                        password_retries=0;
                        password_update_sent=false;
                    }
                    // 每次重新获得修改权限时，只更新一次密钥。
                    if(!serial_info.encrypt_changeable&&buf.radar_info.encrypt_changeable)
                        password_update_sent=false;
                    serial_info.encrypt_level=buf.radar_info.encrypt_level;
                    serial_info.encrypt_changeable=buf.radar_info.encrypt_changeable;
                    serial_info.armor_fortify_active=buf.radar_info.armor_fortify_active;
                    serial_info.armor_fortify_remaining=buf.radar_info.armor_fortify_remaining;
                    serial_info.fortify_center_x=buf.radar_info.fortify_center_x;
                    serial_info.fortify_center_y=buf.radar_info.fortify_center_y;
                    serial_info.fortify_radius=buf.radar_info.fortify_radius;
                }
                printf("Armor fortify: active=%hhu remaining=%hhu center=(%hu,%hu) radius=%hu\n",
                    buf.radar_info.armor_fortify_active,buf.radar_info.armor_fortify_remaining,
                    buf.radar_info.fortify_center_x,buf.radar_info.fortify_center_y,buf.radar_info.fortify_radius);
            break;
            case 0x0301:
                if(!payload_long_enough(buf,sizeof(buf.robot_interaction_data.header)))
                    break;
                printf("Received data from another robot:\ncmd_id: 0x%04hx sender_id: 0x%04hx ",buf.robot_interaction_data.header.data_cmd_id,buf.robot_interaction_data.header.sender_id);
                //比赛模式会关停printf输出
                #ifndef NDEBUG
                printf("data:");
                for(size_t i=sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(buf.robot_interaction_data.header);i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length;++i)
                    printf(" %02hhx",buf.data[i]);
                #endif
                printf("\n");
                switch(buf.robot_interaction_data.header.data_cmd_id)
                {
                    //改一下，没有工程和英雄了。
                    //我决定把所有的装逼的无锁同步功能全部改成有锁的，爱带锁这一块
                    //当然，单个标志位只需要原子操作就行
                    case 0x0279: //Receive ally pos from sentry
                        if(!payload_long_enough(buf,sizeof(buf.robot_interaction_data.header)+sizeof(buf.robot_interaction_data.robot_pos)))
                            break;
                        {
                            std::lock_guard<std::mutex> lock(state_mutex);
                            for(size_t i=0;i<6;++i)
                                if(pos_cache[i+6]>=0&&buf.robot_interaction_data.robot_pos.data[i*2]>0&&buf.robot_interaction_data.robot_pos.data[i*2+1]>0)
                                {
                                    pos_cache[i+6]=0; //Make sure the pos being modified is not used
                                    printf("Robot position updated (%zu)\n",i+6);
                                    pos_from_sentry.data[(i+6)*2]=buf.robot_interaction_data.robot_pos.data[i*2],pos_from_sentry.data[(i+6)*2+1]=buf.robot_interaction_data.robot_pos.data[i*2+1];
                                    pos_cache[i+6]=20; //Override radar's pos for 2 secs
                                }
                        }
                    break;
                }
            break;
            case 0x0A01:
                if(!payload_long_enough(buf,sizeof(buf.radio_position_data)))
                    break;
                // Wire order is 1, reserved, 3, 4, reserved, 7.  Keep the
                // application-facing six-slot layout (hero, engineer,
                // infantry 3/4/5, sentry) and leave the reserved slot empty.
                {
                    const uint16_t wire_pos[12]={
                        buf.radio_position_data.robot_1_x,buf.radio_position_data.robot_1_y,
                        0,0,
                        buf.radio_position_data.robot_3_x,buf.radio_position_data.robot_3_y,
                        buf.radio_position_data.robot_4_x,buf.radio_position_data.robot_4_y,
                        0,0,
                        buf.radio_position_data.robot_7_x,buf.radio_position_data.robot_7_y};
                    std::lock_guard<std::mutex> lock(state_mutex);
                    for(size_t i=0;i<6;++i)
                    if(pos_cache[i]<=0&&wire_pos[i*2]>0&&wire_pos[i*2+1]>0)
                    {
                        pos_cache[i]=0; //Make sure the pos being modified is not used
                        printf("Robot position updated (%zu)\n",i);
                        pos_from_radio.data[i*2]=wire_pos[i*2],pos_from_radio.data[i*2+1]=wire_pos[i*2+1];
                        pos_cache[i]=-20; //Override radar's pos for 2 secs
                    }
                }
            break;
            case 0x0A02:
                if(!payload_long_enough(buf,sizeof(buf.radio_hp_data)))
                    break;
                printf("HP from radio: %hu %hu %hu %hu %hu %hu",
                    buf.radio_hp_data.robot_1,buf.radio_hp_data.reserved_1,
                    buf.radio_hp_data.robot_3,buf.radio_hp_data.robot_4,
                    buf.radio_hp_data.reserved_2,buf.radio_hp_data.robot_7);
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    hp_from_radio=buf.radio_hp_data;
                }
                printf("\n");
            break;
            case 0x0A03:
                if(!payload_long_enough(buf,sizeof(buf.radio_ammo_data)))
                    break;
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    ammo_from_radio=buf.radio_ammo_data;
                }
                printf("Ammo from radio: %hu %hu %hu %hu %hu\n",
                    buf.radio_ammo_data.robot_1,buf.radio_ammo_data.robot_3,
                    buf.radio_ammo_data.robot_4,buf.radio_ammo_data.reserved,
                    buf.radio_ammo_data.robot_7);
            break;
            case 0x0A04:
                if(!payload_long_enough(buf,sizeof(buf.radio_team_status)))
                    break;
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    team_status_from_radio=buf.radio_team_status;
                }
                printf("Team status from radio: coins=%hu total=%hu buffs=0x%08x terrain=0x%08x\n",
                    buf.radio_team_status.remaining_coins,buf.radio_team_status.total_coins,
                    buf.radio_team_status.active_buffs,buf.radio_team_status.terrain_and_control);
            break;
            case 0x0A05:
                if(!payload_long_enough(buf,sizeof(buf.radio_buff_data)))
                    break;
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    buffs_from_radio=buf.radio_buff_data;
                }
                printf("Buff/status from radio updated\n");
            break;
            case 0x0A06:
                if(!payload_long_enough(buf,sizeof(buf.password)))
                    break;
                {
                    // 0x0A06 以 10Hz 重复发送同一个密钥；只有内容变化时才重新发起
                    // 验证，否则不仅刷屏，还会撞上“验证后 10 秒内再次更新无效”的限制。
                    bool changed=false;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        if(memcmp(&guessed_password,&buf.password,sizeof(guessed_password))!=0)
                        {
                            memcpy(&guessed_password,&buf.password,sizeof(guessed_password));
                            password_retries=3;
                            changed=true;
                        }
                    }
                    if(changed)
                    {
                        printf("New opponent password: ");
                        for(size_t i=0;i<sizeof(buf.password)/sizeof(uint8_t);++i)
                            printf("%c",buf.password[i]);
                        printf("\n");
                    }
                }
            break;
            #ifndef NDEBUG
            default:
                fprintf(stderr,"Received an unhandled packet, cmd_id: 0x%04hx\n",buf.cmd_id);
            break;
            #endif
        }
    }
    void msg::receive(serial::stream& src, union frame& buf, std::function<bool(serial::stream&)> reopen)
    {
        ssize_t state;
        while(running)
        {
            while(running&&src.get_fd()>=0)
            {
                // 用 poll 等 SOF：没有数据时也能及时响应 stop()。串口的
                // VMIN/VTIME 只在收到首字节后才计时，直接 read 可能永久阻塞。
                const int fd=src.get_fd();
                if(fd<0)
                    break;
                pollfd waiting{fd,POLLIN,0};
                const int ready=::poll(&waiting,1,200);
                if(ready<0)
                {
                    if(errno==EINTR)
                        continue;
                    break;
                }
                if(ready==0)
                    continue;
                state=src.read(&buf.header,1); //Read SOF
                if(state<=0)
                    break;
                if(buf.header.SOF!=0xA5)
                {
                    #ifndef NDEBUG
                    fprintf(stderr,"Skipping non-header data: %02hhx\n",buf.header.SOF);
                    #endif
                    continue;
                }
                state=src.read_wCRC8(&buf.header,sizeof(buf.header),1); //Read the rest of header
                if(state==serial::stream::kChecksumMismatch)
                    continue; //帧头 CRC8 错，丢弃并重新找 0xA5
                if(state<=0)
                    break;
                else if((state-=static_cast<ssize_t>(sizeof(buf.header)-1))<0) //Read failed
                    continue;
                if(buf.header.data_length>MAX_BUF-(sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(frame_tail)))
                    buf.header.data_length=0; //Prevent overflow
                if(!state||!buf.header.data_length)
                {
                    src.read(&buf.data,sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)); //Skip wrong data
                    continue;
                }
                #ifndef NDEBUG
                fprintf(stderr,"Received header:");
                for(size_t i=0;i<sizeof(buf.header);++i)
                    fprintf(stderr," %02hhx",buf.data[i]);
                fprintf(stderr,"\n");
                #endif
                state=src.read_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail),sizeof(buf.header));
                if(state==serial::stream::kChecksumMismatch)
                {
                    #ifndef NDEBUG
                    fprintf(stderr,"CRC16 mismatch, cmd_id: 0x%04hx\n",buf.cmd_id);
                    #endif
                    continue; //坏帧，重新同步即可，不要断开串口
                }
                if(state<=0)
                    break;
                else if((state-=static_cast<ssize_t>(sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))<0)
                    continue;
                if(!state)
                {
                    #ifndef NDEBUG
                    fprintf(stderr,"Received a wrong packet, cmd_id: 0x%04hx\n",buf.cmd_id);
                    #endif
                    continue;
                }
                referee_callback(buf);
            }
            if(running)
            {
                fprintf(stderr,"Fatal I/O error detected\n");
                if(reopen)
                    while(running&&!reopen(src))
                        std::this_thread::sleep_for(std::chrono::milliseconds(250));
                else
                    return;
            }
        }
    }
    //总发送
    bool msg::referee_send(uint16_t cmd_id, uint16_t data_length, const void* data)
    {
        if((data_length && !data)||data_length>MAX_BUF-sizeof(header)-sizeof(cmd_id)-sizeof(frame_tail))
            return false;
        union frame buf{};
        buf.header={0xA5,data_length,++seq,0};
        buf.cmd_id=cmd_id;
        const size_t buf_len=sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);
        Append_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&buf.header),sizeof(buf.header));
        if(data_length)
            memcpy(buf.data+sizeof(buf.header)+sizeof(buf.cmd_id),data,data_length);
        #ifndef NDEBUG
        fprintf(stderr,"Sending data: ");
        for(size_t i=0;i<buf_len;++i)
            fprintf(stderr,"%02hhx ",buf.data[i]);
        fprintf(stderr,"\n");
        #endif
        return referee.write_wCRC16(&buf.data,buf_len)==static_cast<ssize_t>(buf_len);
    }
    void msg::poll(void)
    {
        while(running)
        {
            for(;;)
            {
                bool ready=false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    ready=serial_info.robot_id!=0;
                }
                if(referee.get_fd()>=0&&ready)
                    break;
                if(!running)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
            // 每轮取一份一致的串口状态快照，避免读到半更新数据。
            struct serial_info state;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                state=serial_info;
            }
            const uint8_t team=state.robot_id/100;
            union both_robot_pos pos{};
            bool known[sizeof(pos)/sizeof(float)]{}; //pos 中哪些槽位本轮有真实值
            timer::time_point target=timer::now()+interval;
            if(timer::now()>timestamp[0])
            {
                size_t count=0;
                int8_t cache[sizeof(pos_cache)/sizeof(int8_t)];
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    for(size_t i=0;i<sizeof(pos_cache)/sizeof(int8_t);++i)
                        if(pos_cache[i]>0&&--pos_cache[i])
                        {
                            //Sentry reports ally positions in metres
                            pos.data[i*2]=pos_from_sentry.data[i*2],pos.data[i*2+1]=pos_from_sentry.data[i*2+1];
                            known[i*2]=known[i*2+1]=true,++count;
                        }
                        else if(pos_cache[i]<0&&++pos_cache[i])
                        {
                            //Radio reports opponent positions in centimetres; convert to metres
                            pos.data[i*2]=static_cast<float>(pos_from_radio.data[i*2])/100.f;
                            pos.data[i*2+1]=static_cast<float>(pos_from_radio.data[i*2+1])/100.f;
                            known[i*2]=known[i*2+1]=true,++count;
                        }
                    for(size_t i=0;i<sizeof(pos_cache)/sizeof(int8_t);++i)
                        cache[i]=pos_cache[i];
                }
                const camera_positions::value_type camera=pos_from_camera.snapshot();
                if(robot_classes.size()>=2)
                    for(size_t i=0;i<camera.second.size()&&i<camera.first.size();++i)
                    {
                        const uint8_t id_from_camera=camera.second[i];
                        if(id_from_camera==(uint8_t)-1)
                            continue;
                        //Slots 0..5 are the opponent half, 6..11 the ally half,
                        //the same convention as 0x0A01 and 0x0279 below.
                        size_t id;
                        auto it=std::find(robot_classes[!team].begin(),robot_classes[!team].end(),id_from_camera);
                        if(it!=robot_classes[!team].end()&&static_cast<size_t>(it-robot_classes[!team].begin())<6)
                            id=static_cast<size_t>(it-robot_classes[!team].begin());
                        else
                        {
                            it=std::find(robot_classes[team].begin(),robot_classes[team].end(),id_from_camera);
                            if(it==robot_classes[team].end())
                                continue;
                            id=6+static_cast<size_t>(it-robot_classes[team].begin());
                            if(id>=12)
                                continue;
                        }
                        if(cache[id])
                            continue;
                        pos.data[id*2]=camera.first[i].x,pos.data[id*2+1]=camera.first[i].y;
                        known[id*2]=known[id*2+1]=true,++count;
                        #ifndef NDEBUG
                        fprintf(stderr,"Robot %zu: (%.2f,%.2f)\n",id,camera.first[i].x,camera.first[i].y);
                        #endif
                    }
                if(persistent_cache||count>0)
                {
                    //Send pos
                    static union both_map_robot buf{}; //Send cached pos when not found
                    if(persistent_cache)
                    {
                        //Legacy guess for slot pair 8/9 based on match time.
                        //Units are metres, same as camera and sentry positions.
                        pos.data[9]=team?15-1.82:1.82;
                        if(state.stage_remain_time<400)
                            pos.data[8]=14;
                        else
                            pos.data[8]=team?400+14-state.stage_remain_time:28-(400+14-state.stage_remain_time);
                        known[8]=known[9]=true;
                    }
                    else memset(&buf,0,sizeof(buf));
                    for(size_t i=0;i<sizeof(pos)/sizeof(float);++i)
                        if(known[i])
                            buf.data[i]=std::max(pos.data[i]*100,1.f); //Convert unit (m->cm) and ensure not overflow
                    if(referee_send(0x0305,sizeof(buf),&buf))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent data to server\n");
                        #endif
                        // 0x0305 协议上限 5Hz（表 1-5），不能按 interval=100ms 发。
                        timestamp[0]+=std::chrono::milliseconds(200);
                    }
                }
            }
            if(timer::now()>timestamp[1])
            {
                const uint8_t radar_cmd_target=std::min(vulnerated.load(),cmd.load());
                const uint8_t armor=armor_fortify.load();
                if(state.game_progress==4&&(radar_cmd_sent<radar_cmd_target||state.vulnerability>0||state.encrypt_changeable||password_retries.load()>0||armor))
                {
                    //Send cmd
                    struct robot_interaction_data buf{};
                    buf.header={0x0121,state.robot_id,0x8080};
                    // 协议要求 radar_cmd 单调且每次只能 +1（表 1-34），
                    // 即使两次发送之间 vulnerated 跳了，也要逐帧补上。
                    if(radar_cmd_sent<radar_cmd_target)
                        ++radar_cmd_sent;
                    buf.cmd.radar_cmd=radar_cmd_sent;
                    const uint8_t retries=password_retries.load();
                    buf.cmd.password_cmd=(retries>0?2:0);
                    if(buf.cmd.password_cmd)
                    {
                        {
                            std::lock_guard<std::mutex> lock(state_mutex);
                            memcpy(&buf.cmd.password,&guessed_password,sizeof(guessed_password));
                        }
                        --password_retries;
                    }
                    else if(state.encrypt_changeable&&!password_update_sent)
                    {
                        // 每次获得修改权限只换一次密钥，避免 10Hz 反复随机换密钥。
                        std::vector<uint8_t> pwd=generate_password(6);
                        memcpy(&buf.cmd.password,pwd.data(),pwd.size());
                        buf.cmd.password_cmd=1;
                        password_update_sent=true;
                    }
                    // armor 先读计数、后读坐标，配合 request_armor_fortify 的写入顺序
                    // 可以保证读到的是同一组请求。
                    buf.cmd.armor_fortify_cmd=armor;
                    buf.cmd.fortify_center_x=fortify_center_x.load();
                    buf.cmd.fortify_center_y=fortify_center_y.load();
                    if(referee_send(0x0301,sizeof(buf.header)+sizeof(buf.cmd),&buf))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent cmd: %hhu %hhu\n",buf.cmd.radar_cmd,buf.cmd.password_cmd);
                        #endif
                        if(buf.cmd.password_cmd)
                        {
                            fprintf(stderr,"%s password: ",buf.cmd.password_cmd==2?"Guessed":"New");
                            for(size_t i=0;i<sizeof(buf.cmd.password)/sizeof(uint8_t);++i)
                                fprintf(stderr,"%c",buf.cmd.password[i]);
                            fprintf(stderr,"\n");
                        }
                        timestamp[1]+=std::chrono::milliseconds(100);
                    }
                }
                if(radio.get_fd()>=0)
                {
                    //Send to radio
                    struct rf_tx_frame buf={0xA5,state.robot_id,state.encrypt_level};
                    if(radio.write(&buf,sizeof(buf))==static_cast<ssize_t>(sizeof(buf)))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent data to radio\n");
                        #endif
                    }
                }
            }
            if(timer::now()>timestamp[2])
            {
                if(std::any_of(pos.opponent.data,pos.opponent.data+12,[](float x){return x!=0;}))
                {
                    struct robot_interaction_data buf{};
                    buf.header={0x0297,state.robot_id,static_cast<uint16_t>(team*100+7)};
                    memcpy(&buf.robot_pos,&pos.opponent,sizeof(pos.opponent));
                    if(referee_send(0x0301,sizeof(buf.header)+sizeof(buf.robot_pos),&buf))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent data to robot\n");
                        #endif
                        timestamp[2]+=std::chrono::milliseconds(50);
                    }
                }
            }
            if(timer::now()<target)
                std::this_thread::sleep_for(target-timer::now());
        }
    }
    bool msg::run(const char* referee_tty, const char* radio_host, uint16_t radio_port)
    {
        // 已经通过构造函数传入 fd 时这里只重新配置 termios，不会重开设备。
        if(!serial_quick_setup(referee,referee_tty))
            return false;
        running=true;
        const std::string referee_path=referee_tty?referee_tty:"";
        const std::string radio_path=radio_host?radio_host:"";
        const uint16_t radio_port_copy=radio_port;
        rx=std::thread(&rm::msg::receive,this,std::ref(referee),std::ref(recv_buf),
            [referee_path](serial::stream& s)
            {
                s.close(); //丢掉坏掉的 fd，再按 run() 传入的路径重开
                return serial_quick_setup(static_cast<serial::terminal&>(s),referee_path.empty()?nullptr:referee_path.c_str());
            }),
            tx=std::thread(&rm::msg::poll,this);
        if(radio.get_fd()<0&&radio_host&&radio_port)
            radio.connect(radio_host,radio_port);
        if(radio_host&&radio_port)
            // 即使首次连接失败也启动接收线程，receive() 会周期重试 connect。
            rf=std::thread(&rm::msg::receive,this,std::ref(radio),std::ref(rf_buf),
                [this,radio_path,radio_port_copy](serial::stream&)
                {
                    std::lock_guard<std::mutex> lock(radio_mutex);
                    return !radio_path.empty()&&radio_port_copy&&radio.connect(radio_path.c_str(),radio_port_copy)==0;
                });
        return true;
    }
    void msg::stop(void)
    {
        running=false;
        // 先 shutdown 唤醒阻塞在 read 上的 TCP 接收线程。
        {
            std::lock_guard<std::mutex> lock(radio_mutex);
            if(radio.get_fd()>=0)
                ::shutdown(radio.get_fd(),SHUT_RDWR);
        }
        // 先 join 再 close：fd 被一个线程 close、另一个线程还在 read 是数据竞争。
        if(tx.joinable())
            tx.join();
        if(rx.joinable())
            rx.join();
        if(rf.joinable())
            rf.join();
        {
            std::lock_guard<std::mutex> lock(radio_mutex);
            radio.close();
        }
        referee.close();
    }
    msg::~msg()
    {
        stop();
    }
}
