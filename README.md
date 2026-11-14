# RoboMaster 2027 雷达站通信

这份代码把上一届 `messenger` 更新到《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0》：

- `0x0003` 按 2027 的 12 个血量/基地/控制度字段解析；
- `0x020C` 解析 16 位被标记进度；
- `0x020E` 解析双倍易伤、干扰波等级和装甲加固状态；
- `0x0301/0x0121` 发送完整 13 字节雷达自主决策指令，支持密钥验证、双倍易伤和装甲加固坐标；
- 接收雷达无线链路的 `0x0A01` 到 `0x0A06`，包括位置、血量、允许发弹量、队伍状态、增益和密钥；
- 保留原来的相机坐标、哨兵坐标、裁判系统小地图上报和 TCP 无线链路接口。

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
| `serial.h` | Linux 串口、PTY 和 TCP 的底层读写实现 | 是 |
| `RM_CRC.h` | 裁判系统 CRC8、CRC16 校验 | 是 |
| `rm2027_protocol.h/.cpp` | 独立的帧编解码器，不依赖串口和 OpenCV | 测试用 |
| `examples/simulated_radar_main.cpp` | 模拟相机坐标和装甲加固的 C++ 主程序 | 示例程序 |
| `tests/test_rm2027_protocol.cpp` | 测试 CRC、帧长度和错误帧检测 | 测试用 |
| `tools/rm2027_referee_sim.py` | 模拟裁判系统，创建 PTY 并发送协议帧 | 测试用 |
| `tools/rm2027_pty_probe.py` | 代替雷达程序连接 PTY，打印收到的帧并回发一帧 | 测试用 |
| `tools/rm2027_protocol.py` | 用命令行生成和解析协议帧 | 调试用 |

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

雷达程序可以通过以下字段发出主动指令；计数值按协议单调递增即可：

```cpp
msg.armor_fortify = next_armor_fortify_counter;
msg.fortify_center_x = x_mm;
msg.fortify_center_y = y_mm;
```

`guessed_password` 和 `password_retries` 仍用于 `0x0A06` 密钥验证流程。正式接入裁判系统前，建议先用 PTY 模拟器确认串口波特率为 115200、8N1，且应用能正确收到 `0x020E` 后再发送 `0x0301`。
