# RoboMaster 2027 雷达站通信

这份代码把上一届 `messenger` 更新到《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（20261029）》，并逐条对照协议表 1-5/1-8/1-11/1-12/1-22/1-24/1-34/1-36/1-41~46 核对过长度和字段：

- `0x0003` 固定 24 字节：己方血量/基地 + 绝对的红/蓝前哨 + 双方基地 + 控制度，己方前哨按 `robot_id` 百位选择；
- `0x0201` 固定 17 字节，包含偏移 12 的 `float bullet_speed_limit`；
- `0x0105` 固定 3 字节，`uint8_t dart_remaining_time + uint16_t dart_info` 位域；
- `0x020C` 固定 2 字节，`uint16_t mark_progress` 标记位图；
- `0x020E` 固定 8 字节，解析双倍易伤、干扰波等级和装甲加固状态；
- `0x0301/0x0121` 发送 13 字节自主决策指令：`radar_cmd` 单调且每次只能 +1，`password_cmd` 为 1（更新密钥）或 2（验证破解的敌方密钥）；
- `0x0305` 固定 48 字节，前 24 字节为对方、后 24 字节为己方，单位 cm，协议上限 5Hz；
- 接收雷达无线链路 `0x0A01~0x0A06`，位置/血量/发弹量单位按协议（位置为 cm）；
- 保留相机坐标、哨兵坐标、小地图上报和 TCP 无线链路接口；
- 所有跨线程共享状态都加了锁或原子量，详见下文“线程模型”。

## 协议长度核对

| 命令号 | 数据长度 | 结构体 | 说明 |
| --- | --- | --- | --- |
| 0x0001 | 11 | `game_status` | 比赛类型/阶段、剩余时间、NTP 时间戳 |
| 0x0002 | 1 | `winner` | 0 平局、1 红胜、2 蓝胜 |
| 0x0003 | 24 | `game_robot_HP` | 12 个 uint16 字段 |
| 0x0104 | 3 | `referee_warning` | 判罚等级、机器人 ID、次数 |
| 0x0105 | 3 | `dart_info` | 剩余时间 + 16 位飞镖信息 |
| 0x0201 | 17 | `robot_status` | 含 `bullet_speed_limit(float)` |
| 0x020C | 2 | `radar_mark_data` | 16 位标记位图 |
| 0x020E | 8 | `radar_info` | 2 字节位域 + 3 个 uint16 |
| 0x0121 | 13 | `radar_command_data` | 0x0301 子内容 |
| 0x0305 | 48 | `both_map_robot` | 对方 24 + 己方 24 |
| 0x0A01/02/03/04/05/06 | 24/12/10/12/47/6 | `radio_*` | 无线链路输入 |

`messenger.h` 里对每个结构体都写了 `sizeof` 注释，并用 `static_assert` 锁定关键长度。

## 文件结构

```text
radar_station_2027/
├── messenger.h
├── messenger.cpp
├── serial.h
├── RM_CRC.h
├── rm2027_protocol.h
├── rm2027_protocol.cpp
├── examples/
│   └── simulated_radar_main.cpp
├── tests/
│   └── test_rm2027_protocol.cpp
├── tools/
│   ├── rm2027_protocol.py
│   ├── rm2027_referee_sim.py
│   ├── rm2027_pty_probe.py
│   └── __pycache__/                 # Python 自动生成，可以忽略
└── build/                           # 编译后生成，可以删除
```

各文件的作用如下。

| 文件 | 用途 | 是否属于正式雷达程序 |
| --- | --- | --- |
| `messenger.h` | 定义 2027 协议数据结构和 `rm::msg` 类 | 是 |
| `messenger.cpp` | 接收裁判系统数据、解析状态、周期发送位置和雷达指令 | 是 |
| `serial.h` | Linux 串口、PTY 和 TCP 的底层读写实现（fd 为原子量） | 是 |
| `RM_CRC.h` | 裁判系统 CRC8、CRC16 校验 | 是 |
| `rm2027_protocol.h/.cpp` | 独立的帧编解码器，不依赖串口和 OpenCV | 测试用 |
| `examples/simulated_radar_main.cpp` | 模拟相机坐标和装甲加固的 C++ 主程序 | 示例程序 |
| `tests/test_rm2027_protocol.cpp` | 测试 CRC、帧长度和错误帧检测 | 测试用 |
| `tools/rm2027_referee_sim.py` | 模拟裁判系统，创建 PTY 并发送协议帧 | 测试用 |
| `tools/rm2027_pty_probe.py` | 代替雷达程序连接 PTY，打印收到的帧并回发一帧 | 测试用 |
| `tools/rm2027_protocol.py` | 用命令行生成和解析协议帧 | 调试用 |
| `fusion/` | 激光雷达 + 相机（+ 可选无线电）融合模块，独立于 ROS 和通信层 | 可选 |

完整雷达程序的依赖关系是：

```text
simulated_radar_main.cpp 或你自己的 main.cpp
                    │
                    ▼
            messenger.h/.cpp
              │          │
              ▼          ▼
          serial.h     RM_CRC.h
              │
              ├── Linux 串口或 PTY
              └── TCP 无线链路
```

协议单元测试的依赖关系更简单：

```text
test_rm2027_protocol.cpp
              │
              ▼
rm2027_protocol.h/.cpp
```

它不会使用 `messenger.cpp`、`serial.h` 或 OpenCV。

`build/` 只是编译输出目录，里面的文件可以随时重新生成。`tools/__pycache__/` 是 Python 自动生成的缓存目录，不需要手动编辑。

`rm2027_protocol.h/.cpp` 是不依赖串口库和 OpenCV 的帧编解码器，适合在 PC 上先做 CRC 和帧长度测试。它使用协议附录中的 CRC8/CRC16 参数：CRC8 初值 `0xff`、多项式 `0x31`，CRC16 初值 `0xffff`、反射多项式 `0x8408`。

## 线程模型与加锁

`rm::msg` 内部有 3 条线程：

- `rx`：读裁判系统串口；
- `tx`：`poll()`，周期发送 0x0305/0x0301 并写 TCP 无线帧；
- `rf`：读 TCP 无线链路（传入 host/port 时才会启动，断线后自动重连）。

同步规则：

- **多字段状态用 `state_mutex`**：`serial_info`、`pos_from_sentry`、`pos_from_radio`、`hp/ammo/team_status/buffs_from_radio`、`pos_cache`、`guessed_password`。发送线程每轮先取一份 `serial_info` 快照，整轮使用同一份数据。
- **单标志/计数用原子量**：`running`、`cmd`、`pre_dart_target`、`password_retries`、`vulnerated`、`radar_cmd_sent`、`password_update_sent`、`armor_fortify`、`fortify_center_x/y`。
- **相机坐标用 `camera_positions`**：视觉线程写、发送线程读，内部互斥并返回快照。
- **TCP 用 `radio_mutex`**：保护 `connect` 与 `stop()` 里的 `shutdown/close`。
- **`serial.h` 的 `fd_` 是 `std::atomic<int>`**：关闭和读写可以并发。
- **退出顺序**：`stop()` 置 `running=false` → `shutdown` 唤醒 TCP 读 → join 所有线程 → 最后 close fd。
- 接收线程等 SOF 时用 `poll(..., 200ms)`，没有数据也能及时响应 `stop()`。
- CRC8/CRC16 校验失败返回 `kChecksumMismatch`，接收线程丢弃坏帧重新同步，不会为了一个坏帧重开串口。

## 对外接口

视觉线程整体替换相机检测结果（保留原来的 `pair` 赋值写法，内部会加锁）：

```cpp
msg.pos_from_camera = std::make_pair(points, ids);
```

装甲加固请求建议一次性写入，避免发送线程读到“新计数配旧坐标”：

```cpp
msg.request_armor_fortify(next_armor_fortify_counter, x_mm, y_mm);
```

原来的 `msg.armor_fortify = ...; msg.fortify_center_x = ...;` 也仍然可用（成员是原子量），但一次写一组时推荐用上面的函数。

显示/决策线程需要多字段一致时，取快照而不是直接读成员：

```cpp
auto state = msg.get_serial_info();
// state.game_progress、state.stage_remain_time、state.vulnerability ...
```

直接读 `msg.serial_info.xxx` 单个字段仍然兼容。

`0x0301` 的 `0x0279`（哨兵发来的己方坐标）和 `0x0297`（发给哨兵的位置）属于协议里 `0x0200~0x02FF` 的机器人间自定义通信，内容和结构由队内约定。

## 激光雷达 + 相机融合（fusion/）

`fusion/` 是独立的融合模块，不依赖 ROS/OpenCV，也不修改 `radar/radar_docker`：

- 激光 `/learning/tracks`（轨迹号、位置、速度，无身份）经 `fusion/tools/lidar_udp_bridge.py`
  用 UDP 送给融合程序；
- 相机沿用上一届的 `center_points + ids`（场地坐标、米）；
- 无线电 0x0A01/解波可选作为第三路兜底。

融合把相机身份绑定到激光轨迹：两者都在时输出稳定位置，相机短时丢失时靠激光延续身份，
只有相机时退化为上一届行为。构建、UDP 协议、docker 桥接和接入相机程序的说明见
[`fusion/README.md`](fusion/README.md)。

```bash
./fusion/scripts/run.sh build          # 构建
./fusion/scripts/run.sh demo 10        # 无硬件端到端演示
./fusion/scripts/run.sh rviz-calib     # 带 Publish Point 的 RViz（标定用）
./fusion/scripts/run.sh calibrate-live # 点击标定激光->场地外参
FUSION_CALIB=fusion/tmp/field_calibration.json ./fusion/scripts/run.sh all
```

`camera_app/` 是上一届 `radar_station` 的副本，融合已经接入（原目录未修改）。
海康 MV-CA016-10UC 由原有 Aravis 后端按序列号打开，可以用一条命令启动
激光桥 + 相机程序：

```bash
./camera_app/run.sh build
FUSION_CALIB=fusion/tmp/field_calibration.json ./camera_app/run.sh fusion
```

相机标定直接调用复制过来的原有标定程序（`./camera_app/run.sh calibrate ...`），
详见 [`camera_app/FUSION.md`](camera_app/FUSION.md)。

## 离线测试

```bash
mkdir -p build

g++ -std=c++17 -Wall -Wextra -I. \
    tests/test_rm2027_protocol.cpp rm2027_protocol.cpp \
    -o build/test_rm2027_protocol
build/test_rm2027_protocol

python3 tools/rm2027_protocol.py build \
    --cmd 0x020e --payload-hex 0101000000000000 --seq 3
```

## PTY 裁判系统模拟器

模拟器会创建一个伪串口，周期发送比赛状态、雷达状态和 `0x0A01~0x0A06`，并校验雷达程序发回的每一帧：

```bash
python3 tools/rm2027_referee_sim.py --seconds 60
# 另一个终端，把脚本输出的 /dev/pts/N 作为 referee_tty 传给雷达程序
```

实际操作顺序是：

1. 终端 A 启动模拟器，复制它打印的 `/dev/pts/N`；
2. 终端 B 启动雷达程序，并把这个路径传给 `msg::run()`；
3. 回到终端 A 按回车，模拟器才开始发帧；
4. 雷达程序收到 `0x0001`、`0x020E` 等数据，模拟器终端会打印雷达发出的 `TX cmd=...`。

注意：一定要等雷达程序打开 PTY、把串口设置成 raw 之后再让模拟器发帧。如果模拟器提前发帧，串口回显会把模拟器自己的接收缓冲污染成乱码（自动化脚本里可以让模拟器的 stdin 先空等几秒）。

如果完整雷达程序还没有可执行文件，可以先用最小探针验证 PTY 双向通信：

```bash
python3 tools/rm2027_pty_probe.py /dev/pts/N --seconds 10
```

探针会读取模拟器发来的帧，并主动回发一个合法的 `0x0301/0x0121` 空指令；模拟器终端应显示一行 `TX cmd=0x0301`。

仓库还提供了带注释的 C++ 主程序示例：[simulated_radar_main.cpp](</home/mindu/radar_station_2027/examples/simulated_radar_main.cpp>)。它模拟相机坐标，并在第 3 秒和第 10 秒分别请求一次装甲加固。当前仓库已经包含 `serial.h` 和 `RM_CRC.h`，可以直接编译：

```bash
g++ -std=c++17 -Wall -Wextra -O2 \
    -I. \
    $(pkg-config --cflags opencv4) \
    examples/simulated_radar_main.cpp \
    messenger.cpp \
    $(pkg-config --libs opencv4) -pthread \
    -o build/simulated_radar
```

运行方式：

```bash
./build/simulated_radar /dev/pts/N 60
```

正常运行时应该看到：

- 模拟器打印 `TX cmd=0x0305`，约 5Hz，前 24 字节是无线链路给的对方坐标（cm），后 24 字节是相机给的己方坐标；
- `TX cmd=0x0301`，子内容 `0x0121` 为 19 字节、`0x0297` 为 54 字节；
- 雷达端 0 次 `CRC16 mismatch`、0 次 `Fatal I/O error`。

## ThreadSanitizer

跨线程改动后建议跑一次 TSan：

```bash
g++ -std=c++17 -g -O1 -fsanitize=thread \
    -I. $(pkg-config --cflags opencv4) \
    examples/simulated_radar_main.cpp messenger.cpp \
    $(pkg-config --libs opencv4) -pthread \
    -o build/simulated_radar_tsan

setarch $(uname -m) -R ./build/simulated_radar_tsan /dev/pts/N 30
```

如果启动时报 `ThreadSanitizer: unexpected memory mapping`，就是内核 ASLR 随机化太强，用上面的 `setarch -R` 关掉地址随机化即可。当前主链路实测 0 条 TSan 警告。

## 接入实车

正式接入裁判系统前，建议先用 PTY 模拟器确认：

1. 串口为 115200、8N1，应用能正确收到 `0x0201` 和 `0x020E`；
2. `0x020E` 的 `encrypt_changeable` 为 1 时，程序只换一次密钥（`password_cmd=1`）；
3. `0x0A06` 的敌方密钥变化时，程序用 `password_cmd=2` 验证；
4. 双倍易伤请求按 `radar_cmd` 逐次 +1 发出，装甲加固按单调计数发出；
5. 0x0305 不超过 5Hz、0x0301 总体不超过 30Hz。
