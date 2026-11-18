# 激光雷达 + 相机融合

一套可以直接启动的融合系统：

- **激光**：`radar_learning` 的 `/learning/tracks`（轨迹号、位置、速度，无身份）；
- **相机**：上一届 `radar_station` 的车辆/装甲检测（类别号、场地坐标，米）；
- **无线电 0x0A01 / 解波**：可选，带槽位坐标，做兜底；
- **输出**：带身份、位置、速度、来源、置信度的 `FusedTarget` 列表；相机程序可以直接把它转成
  `msger.pos_from_camera`。

融合核心不依赖 ROS/OpenCV；相机适配器 `CameraFusion` 依赖 OpenCV；ROS 桥接脚本在现有
radar docker 里运行，**不修改 `radar/radar_docker` 仓库**。

## 交付内容

```text
fusion/
├── include/fusion/
│   ├── types.h            # 数据结构、外参、参数
│   ├── fusion.h           # 融合核心
│   └── camera_fusion.h    # 相机程序直接链接的适配器（方案 A）
├── src/                   # 对应实现
├── tools/
│   ├── calibrate_field.py # 标定程序（CSV / RViz 点击）
│   ├── lidar_udp_bridge.py# ROS /learning/tracks -> UDP
│   ├── fusion_demo.cpp    # 合成场景验证
│   ├── fusion_udp_listener.cpp # 监听 UDP 并打印融合结果
│   └── udp_demo_sender.py # 无硬件时的合成 UDP 数据
├── scripts/run.sh         # 一键构建/启动/标定
├── config/                # 融合参数 + 外参格式示例
├── tests/                 # 融合 + 相机适配器单元测试
└── tmp/                   # 临时文件，已 gitignore
```

## 快速开始

```bash
# 1. 构建
./fusion/scripts/run.sh build

# 2. 不需要任何硬件，先看端到端效果（合成相机 + 合成激光）
./fusion/scripts/run.sh demo 10

# 3. 标定示例（自洽数据，验证工具链）
./fusion/scripts/run.sh calibrate-example
./fusion/scripts/run.sh calibrate-csv fusion/tmp/example_points.csv

# 4. 实车标定（需要雷达 docker 已 up）
./fusion/scripts/run.sh rviz-calib     # 终端 2：带 Publish Point 的 RViz
./fusion/scripts/run.sh calibrate-live # 终端 3：点击并输入场地坐标

# 5. 实车运行：ROS 激光桥 + 融合监听（打印融合结果）
FUSION_CALIB=fusion/tmp/field_calibration.json ./fusion/scripts/run.sh all

# 6. 相机程序已接好融合后，一条命令启动“激光桥 + 相机程序”
FUSION_CALIB=fusion/tmp/field_calibration.json ./fusion/scripts/run.sh camera <相机参数>
```

`all` 需要雷达 ROS 栈在运行：

```bash
cd /home/mindu/radar/radar_docker
./radar.sh up
```

激光桥依赖 `dev_ws/devel/setup.bash`（`radar_learning` 的 `TargetStateArray` 已经编译）。
如果不存在，先在 `./radar.sh dev` 里 `catkin_make`。

## 标定程序

`fusion/tools/calibrate_field.py` 求 `field = R * lidar + t`，输出 JSON 可直接给融合：

```json
{
  "rotation": [r00, r01, r02, r10, r11, r12, r20, r21, r22],
  "translation": [tx, ty, tz],
  "lidar_frame": "ws_scan",
  "field_frame": "field"
}
```

场地系与 0x0305 一致：**红方补给区角落为原点，X 向蓝方，Y 向红方停机坪，单位米**。
标定点要选雷达能稳定看到、且场地坐标已知的标识（场地角点、固定柱/围挡交点等）。

### 在线点击（推荐）

`./radar.sh rviz` 使用的 `container.rviz` 里 Tools 只有 `Interact / MoveCamera / Select`，
所以工具栏里**没有 Publish Point**。用下面这个命令打开标定专用 RViz：

```bash
# 终端 1：确认雷达已启动
cd /home/mindu/radar/radar_docker && ./radar.sh up

# 终端 2：带 Publish Point 的 RViz（配置在 fusion/config/calibration.rviz）
./fusion/scripts/run.sh rviz-calib

# 终端 3：标定程序
./fusion/scripts/run.sh calibrate-live
```

`rviz-calib` 会复用 `radar_docker` 的 GUI 启动方式，只额外挂载 fusion 目录和 RViz 配置，
不修改 `radar_docker` 仓库。工具栏里应该能看到 **Publish Point**（点状准星图标），
点击后发布到 `/clicked_point`。

如果你想继续用原来的 RViz 窗口，也可以把下面这段加到自己的 `.rviz` 配置的
`Visualization Manager:` 下并重启 RViz：

```yaml
  Tools:
    - Class: rviz/Interact
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
```

然后在 RViz 里点一个标识点，终端会打印它的雷达坐标，输入它的场地坐标：

```text
点击点 (雷达系): 1.2345 -0.5678 0.0000
场地系 x y z [fit|check]（回车结束）: 0 0 0 fit
```

至少 4 个 `fit`、2 个 `check`；直接回车结束并写出
`fusion/tmp/field_calibration.json`。误差超过 `--max-error`（默认 0.05 m）会拒绝写入，
`--force` 可强制。

### 手动输入（不需要 RViz）

如果你已经知道每个标识点的雷达坐标和场地坐标，可以直接在终端输入：

```bash
./fusion/scripts/run.sh calibrate-manual
```

每行格式（空格或逗号分隔，最后可选 `fit|check`，默认 `fit`）：

```text
雷达 lx ly lz    场地 fx fy fz    [fit|check]
例如: 1.23 -0.45 0.00   0.00 0.00 0.00 fit
```

直接回车结束。雷达坐标可以从雷达容器里看：

```bash
docker exec -it radar-ros1-radar-1 bash -lc '
  source /opt/ros/noetic/setup.bash &&
  rostopic echo /learning/tracks'
```

（`radar-ros1-radar-1` 是 `./radar.sh up` 起的容器名；也可以用 `./radar.sh shell`。）

### 离线 CSV

```text
role,id,lx,ly,lz,fx,fy,fz
fit,corner_a,...(雷达 x y z)...,...(场地 x y z)...
check,post_1,...
```

```bash
./fusion/scripts/run.sh calibrate-csv fusion/tmp/points.csv fusion/tmp/field_calibration.json
# 覆盖已有输出时追加 --force
./fusion/scripts/run.sh calibrate-csv fusion/tmp/points.csv fusion/tmp/field_calibration.json --force
```

### 自检

```bash
python3 fusion/tools/calibrate_field.py --self-test
```

## 相机接入（方案 A，已集成在 camera_app/）

`camera_app/` 是上一届 `radar_station` 的完整副本，**`radar_station` 原目录没有被修改**。
副本里已经完成三处融合接入：

- `camera_app/CMakeLists.txt` 链接本仓库的 `fusion_camera`；
- `camera_app/src/main.cpp` 定义 `fusion::CameraFusion`，按 `FUSION_CALIB`、
  `FUSION_CONFIG`、`FUSION_PORT` 初始化；
- `output()` 每帧调用 `g_fusion.update(center_points, ids)`，返回值赋给
  `msger.pos_from_camera`（包括没有检测的空列表，保证丢失计时推进）。

`CameraFusion` 已经处理上一届代码的细节：`ids` 里的 `-1` 实际是 `uint8_t` 的 255，
会被丢弃；相机线程和 UDP 激光线程由内部互斥量保护；返回值仍是
`(vector<cv::Point2f>, vector<uint8_t>)`。

构建与运行：

```bash
./camera_app/run.sh build
./camera_app/run.sh run        # 默认 -V=-2 -m=15 -s=/dev/ttyUSB0 00J17982005

# 激光桥 + 相机融合
FUSION_CALIB=fusion/tmp/field_calibration.json ./camera_app/run.sh fusion
# 纯相机（不启激光桥）
FUSION_START_BRIDGE=0 ./camera_app/run.sh fusion
```

相机内参标定直接调用复制过来的**原有标定程序**，不重新实现：

```bash
./camera_app/run.sh calibrate -w=11 -h=8 -s=0.2 -n=50 -d=500 \
    -o=camera.yml -op -oe 0
```

海康 MV-CA016-10UC 通过现有 Aravis 后端按序列号 `00J17982005` 打开
（`-V=-2`）；程序使用相机上报的原生分辨率，不强制 640×480。
原有 Bayer 转换和推理链路保持不变。

详细说明见 [`camera_app/FUSION.md`](../camera_app/FUSION.md)。

### 相机类别与槽位

融合把相机 `label` 原样透传。上一届 `robot_classes` 是
`{{0,1,2,3,254,5},{6,7,8,9,254,11}}`（类别号，不是机器人 ID），
最终映射到 0x0305 槽位仍由 `messenger` 完成；换 2027 模型时只需要保持
“相机输出类别号 ↔ `robot_classes`”一致。

## 融合规则

1. 每帧用相机位置和激光轨迹做最近邻关联（`gate` 默认 1.2 m），
   已绑定的轨迹有 `switch_penalty` 惩罚，减少跳变。
2. 相机和激光都新鲜：位置按 `lidar_weight`（默认 0.7）加权，速度取激光。
3. 相机丢失：用绑定激光轨迹的外推位置继续输出身份，最多 `identity_ttl`（默认 2 s），
   置信度衰减。
4. 没有激光的相机目标：按相机速度外推最多 `camera_hold`（默认 0.8 s）。
5. 没有身份的激光轨迹：`label=-1` 输出（`include_unlabeled`）。
6. `use_radio=true` 时，附近没有相机/激光结果的无线电槽位作为 `Source::Radio` 输出。

参数在 `config/fusion.conf`，所有距离为米、时间为秒。

## UDP 协议

文本，每行一条，一个 datagram 可以带多行：

```text
L <id> <x> <y> <z> <vx> <vy> <vz> <predicted>   # 激光，ws_scan 系，米
C <label> <x> <y> <confidence>                  # 相机，场地系，米
R <slot> <x_cm> <y_cm> <valid>                  # 无线电，场地系，厘米
```

单独跑融合监听：

```bash
FUSION_CALIB=fusion/tmp/field_calibration.json ./fusion/scripts/run.sh fusion
```

没有硬件时验证 UDP：

```bash
./fusion/scripts/run.sh demo 10
```

## 验证结果

```text
$ ./fusion/build/fusion_tests
fusion tests passed

$ ./fusion/build/camera_fusion_tests
camera fusion tests passed

$ ./fusion/scripts/run.sh demo 10
仿真 12.0s，相机 30Hz，激光 10Hz，场地 28x15 m
label  输出率  RMSE(m)  遮挡帧 遮挡输出率
101      100.0%    0.058     90(1.5s)   100.0% (RMSE 0.043)
102      100.0%    0.067          -          -
103      100.0%    0.059          -          -
104      100.0%    0.059          -          -
```

（`fusion_demo` 是 4 目标绕圈场景；101 相机遮挡 1.5s 靠激光保持身份，
102 激光遮挡期间退化为纯相机。）

## 限制与注意事项

- 关联是贪心最近邻，没有协方差/Hungarian；目标密集时建议升级。
- `lidar_weight`、`gate`、`identity_ttl` 需要按实车标定和噪声调整。
- UDP 融合用“收到数据的时刻”做时间基准，没有用 ROS/相机时间戳。
- 同一帧同一类别出现多次时只保留最后一次，与 0x0305 每机器人一个位置的语义一致。
- 无线电目前只做兜底，没有参与身份绑定。
- `fusion/tmp/` 放临时数据、日志、标定文件，已被 gitignore。
