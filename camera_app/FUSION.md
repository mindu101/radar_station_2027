# camera_app：上一届相机程序（本仓库内的副本）

这个目录是 `/home/mindu/radar_station` 的完整副本，原目录**没有被修改**。
所有改动只发生在这份副本里：

1. `CMakeLists.txt` 链接本仓库的 `fusion_camera`；
2. `src/main.cpp` 顶部包含 `fusion/camera_fusion.h`，定义 `g_fusion`；
3. `main()` 在串口启动后按 `FUSION_CALIB/FUSION_CONFIG/FUSION_PORT` 初始化融合；
4. `output()` 每帧把 `center_points/ids` 交给 `g_fusion.update()`，返回值赋给
   `msger.pos_from_camera`。

原有相机采集、Bayer 转换、推理、跟踪、标定和通信逻辑都没有改。

## 海康相机

相机已由本机 Aravis 枚举：`Hikrobot-2BDF17982005-00J17982005`，USB3，序列号
`00J17982005`。原有 Aravis 后端可以直接使用：

```bash
./camera_app/run.sh probe          # 应看到 Hikrobot ... (USB3)
./camera_app/run.sh build
./camera_app/run.sh run            # 默认 -V=-2 -m=15 -s=/dev/ttyUSB0 00J17982005
```

`-V=-2` 表示按序列号打开；`-V=2 0` 表示按索引打开（只接一台时也可用）。
程序使用相机自己上报的分辨率，不强制 640×480。

## 标定（调用原有标定程序）

这里不重新实现相机标定，直接调用本目录复制并编译的上一届标定程序：

```bash
# 板子参数按实际棋盘格填写；最后的 0 是 Aravis 设备索引
./camera_app/run.sh calibrate -w=11 -h=8 -s=0.2 -n=50 -d=500 \
    -o=camera.yml -op -oe 0
```

输出写在本目录的 `camera.yml`。运行 `radar` 时也用这份文件（默认 `-M=camera.yml`）。
如果换了分辨率/镜头，就重新跑一次上面的命令；不要依赖仓库里旧的 640×480 文件。

场地 PnP 标记仍由原有流程完成（`-m=7` 启动后按提示标记），结果写 `config.yml`。

## 融合运行

```bash
# 1) 激光->场地外参（fusion 自带的标定工具）
./fusion/scripts/run.sh calibrate-manual
# 或 RViz 点击：./fusion/scripts/run.sh rviz-calib + calibrate-live

# 2) 相机 + 激光融合（脚本会先起 ROS 激光桥）
FUSION_CALIB=fusion/tmp/field_calibration.json \
    ./camera_app/run.sh fusion

# 不想启激光桥（纯相机）：
FUSION_START_BRIDGE=0 ./camera_app/run.sh fusion
```

融合参数和外参：

- `FUSION_CALIB`：激光→场地 JSON（没有则激光按同坐标系处理并警告）；
- `FUSION_CONFIG`：默认 `fusion/config/fusion.conf`；
- `FUSION_PORT`：UDP 端口，默认 11411；设为 0 可关闭激光输入。

相机 `ids` 里的 255 哨兵、以及“每帧都要 update”的细节，`CameraFusion` 已经处理。
