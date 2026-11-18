#!/usr/bin/env bash
# 融合系统的一键入口。
#
#   ./fusion/scripts/run.sh build             构建融合库/工具
#   ./fusion/scripts/run.sh demo [秒数]       合成数据端到端演示（不需要硬件）
#   ./fusion/scripts/run.sh fusion            只启动融合监听（打印结果）
#   ./fusion/scripts/run.sh all               启动 ROS 激光桥 + 融合监听
#   ./fusion/scripts/run.sh bridge            只启动 ROS 激光桥（前台）
#   ./fusion/scripts/run.sh calibrate-live    RViz 点击标定（在雷达 docker 里）
#   ./fusion/scripts/run.sh calibrate-csv <csv> [out.json]
#   ./fusion/scripts/run.sh calibrate-example [csv]
#   ./fusion/scripts/run.sh camera [相机程序参数...]  启动激光桥 + 已接入融合的相机程序
#
# camera 命令默认找 /home/mindu/radar_station/build/bin/radar，
# 可用 FUSION_CAMERA_BIN 指定；FUSION_START_BRIDGE=0 可跳过激光桥。
#
# 常用环境变量：
#   FUSION_PORT=11411
#   FUSION_CALIB=/path/to/field_calibration.json
#   FUSION_CONFIG=fusion/config/fusion.conf
#   RADAR_IMAGE=radar-et3:noetic
#   RADAR_DEV_WS=/home/mindu/radar/radar_docker/dev_ws
#   ROS_MASTER_URI=http://127.0.0.1:11321
#   ROS_IP=127.0.0.1

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FUSION_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$FUSION_DIR/build"
LISTENER="$BUILD_DIR/fusion_udp_listener"
BRIDGE_NAME="fusion-lidar-bridge"

PORT="${FUSION_PORT:-11411}"
CALIB="${FUSION_CALIB:-}"
CONFIG="${FUSION_CONFIG:-$FUSION_DIR/config/fusion.conf}"
IMAGE="${RADAR_IMAGE:-radar-et3:noetic}"
DEV_WS="${RADAR_DEV_WS:-/home/mindu/radar/radar_docker/dev_ws}"
MASTER="${ROS_MASTER_URI:-http://127.0.0.1:11321}"
ROSIP="${ROS_IP:-127.0.0.1}"
RADAR_DIR="${RADAR_DIR:-/home/mindu/radar/radar_docker}"
CALIB_OUT="${FUSION_CALIB_OUT:-$FUSION_DIR/tmp/field_calibration.json}"
CHILD_PID=""
BRIDGE_PID=""

cleanup() {
    if [[ -n "$CHILD_PID" ]]; then
        kill "$CHILD_PID" 2>/dev/null || true
        wait "$CHILD_PID" 2>/dev/null || true
        CHILD_PID=""
    fi
    docker rm -f "$BRIDGE_NAME" >/dev/null 2>&1 || true
}

usage() {
    cat <<'EOF'
用法: ./fusion/scripts/run.sh <命令>

  build                     构建融合库/工具
  demo [秒数]               合成数据端到端演示（不需要硬件）
  fusion                    只启动融合监听并打印结果
  all                       启动 ROS 激光桥 + 融合监听
  bridge                    只启动 ROS 激光桥（前台）
  rviz-calib                打开带 Publish Point 的 RViz（标定用）
  calibrate-live            RViz 点击标定（在雷达 docker 里，交互式）
  calibrate-csv <csv> [out] [--force 等额外参数]  离线 CSV 标定
  calibrate-manual          终端手输雷达坐标 + 场地坐标（不需要 RViz）
  calibrate-example [csv]   生成一份自洽的示例 CSV
  camera [参数...]          启动激光桥 + 已接入融合的相机程序

环境变量: FUSION_PORT, FUSION_CALIB, FUSION_CONFIG, FUSION_CAMERA_BIN,
          FUSION_START_BRIDGE=0 可跳过激光桥,
          RADAR_IMAGE, RADAR_DEV_WS, ROS_MASTER_URI, ROS_IP
EOF
}

build() {
    cmake -S "$FUSION_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" -j"$(nproc 2>/dev/null || echo 2)"
}

ensure_build() {
    if [[ ! -x "$LISTENER" ]]; then
        echo "首次运行，先构建 fusion ..."
        build
    fi
}

listener_args() {
    LISTENER_ARGS=(--port "$PORT" --config "$CONFIG")
    if [[ -n "$CALIB" ]]; then
        LISTENER_ARGS+=(--calib "$CALIB")
    else
        echo "提示: 未设置 FUSION_CALIB，激光按与场地同坐标系处理" >&2
    fi
}

check_dev_ws() {
    if [[ ! -f "$DEV_WS/devel/setup.bash" ]]; then
        echo "找不到 $DEV_WS/devel/setup.bash" >&2
        echo "先在雷达学习容器里构建 radar_learning：" >&2
        echo "  cd /home/mindu/radar/radar_docker && ./radar.sh dev" >&2
        echo "  catkin_make" >&2
        exit 1
    fi
}

start_bridge() {
    check_dev_ws
    docker rm -f "$BRIDGE_NAME" >/dev/null 2>&1 || true
    docker run --rm --name "$BRIDGE_NAME" --network host \
        -e ROS_MASTER_URI="$MASTER" -e ROS_IP="$ROSIP" \
        -v "$DEV_WS:/work/dev_ws:ro" \
        -v "$FUSION_DIR:/fusion:ro" \
        "$IMAGE" bash -lc '
            source /opt/ros/noetic/setup.bash &&
            source /work/dev_ws/devel/setup.bash &&
            exec python3 /fusion/tools/lidar_udp_bridge.py --host 127.0.0.1 --port '"$PORT" &
    BRIDGE_PID=$!
}

run_fusion() {
    ensure_build
    listener_args
    exec "$LISTENER" "${LISTENER_ARGS[@]}"
}

run_all() {
    ensure_build
    listener_args
    start_bridge
    trap cleanup EXIT INT TERM
    sleep 1
    "$LISTENER" "${LISTENER_ARGS[@]}"
}

run_bridge() {
    check_dev_ws
    exec docker run --rm --name "$BRIDGE_NAME" --network host \
        -e ROS_MASTER_URI="$MASTER" -e ROS_IP="$ROSIP" \
        -v "$DEV_WS:/work/dev_ws:ro" \
        -v "$FUSION_DIR:/fusion:ro" \
        "$IMAGE" bash -lc '
            source /opt/ros/noetic/setup.bash &&
            source /work/dev_ws/devel/setup.bash &&
            exec python3 /fusion/tools/lidar_udp_bridge.py --host 127.0.0.1 --port '"$PORT"
}

run_demo() {
    ensure_build
    local seconds="${1:-15}"
    listener_args
    mkdir -p "$FUSION_DIR/tmp"
    local log="$FUSION_DIR/tmp/demo_listener.log"
    rm -f "$log"
    "$LISTENER" "${LISTENER_ARGS[@]}" >"$log" 2>&1 &
    CHILD_PID=$!
    trap cleanup EXIT INT TERM
    sleep 0.5
    python3 "$FUSION_DIR/tools/udp_demo_sender.py" --port "$PORT" --seconds "$seconds"
    sleep 0.3
    echo
    echo "融合输出（$log）:"
    tail -n 20 "$log"
    cleanup
    trap - EXIT INT TERM
}

run_camera() {
    local bin="${FUSION_CAMERA_BIN:-/home/mindu/radar_station_2027/camera_app/build/bin/radar}"
    if [[ ! -x "$bin" ]]; then
        echo "找不到相机程序 $bin" >&2
        echo "先运行 ./camera_app/run.sh build，或用 FUSION_CAMERA_BIN 指定路径。" >&2
        exit 1
    fi
    if [[ "${FUSION_START_BRIDGE:-1}" != "0" ]]; then
        start_bridge
        trap cleanup EXIT INT TERM
        sleep 1
    fi
    local env_args=(FUSION_PORT="$PORT" FUSION_CONFIG="$CONFIG")
    [[ -n "$CALIB" ]] && env_args+=(FUSION_CALIB="$CALIB")
    env "${env_args[@]}" "$bin" "$@"
}

rviz_calib() {
    if [[ -z "${DISPLAY:-}" ]] || ! command -v xhost >/dev/null 2>&1; then
        echo "请在主机桌面终端运行；需要 DISPLAY 和 xhost（Ubuntu 包 x11-xserver-utils）。" >&2
        exit 1
    fi
    if [[ ! -f "$RADAR_DIR/compose.yaml" ]]; then
        echo "找不到 $RADAR_DIR/compose.yaml" >&2
        exit 1
    fi
    if [[ -z "$(cd "$RADAR_DIR" && docker compose --project-directory "$RADAR_DIR" -f compose.yaml ps --status running -q radar)" ]]; then
        echo "请先启动雷达：cd $RADAR_DIR && ./radar.sh up" >&2
        exit 1
    fi
    xhost +si:localuser:root
    (cd "$RADAR_DIR" && docker compose --project-directory "$RADAR_DIR" \
        -f compose.yaml -f compose.gui.yaml \
        run --rm --no-deps -v "$FUSION_DIR:/fusion:ro" radar \
        bash /opt/radar-rviz/launch.sh -d /fusion/config/calibration.rviz)
}

calibrate_live() {
    mkdir -p "$(dirname "$CALIB_OUT")"
    docker run --rm -it --network host \
        -e ROS_MASTER_URI="$MASTER" -e ROS_IP="$ROSIP" \
        -v "$FUSION_DIR:/fusion:ro" \
        -v "$(dirname "$CALIB_OUT"):/data" \
        "$IMAGE" bash -lc '
            source /opt/ros/noetic/setup.bash &&
            exec python3 /fusion/tools/calibrate_field.py --live --output /data/'"$(basename "$CALIB_OUT")"
}

calibrate_csv() {
    local csv="${1:?用法: run.sh calibrate-csv <csv> [out.json] [额外参数]}"
    shift
    local output="$CALIB_OUT"
    if [[ $# -gt 0 && "$1" != --* ]]; then
        output="$1"
        shift
    fi
    python3 "$FUSION_DIR/tools/calibrate_field.py" --csv "$csv" --output "$output" "$@"
}

calibrate_manual() {
    python3 "$FUSION_DIR/tools/calibrate_field.py" --manual \
        --output "$CALIB_OUT" "$@"
}

calibrate_example() {
    local csv="${1:-$FUSION_DIR/tmp/example_points.csv}"
    mkdir -p "$(dirname "$csv")"
    python3 "$FUSION_DIR/tools/calibrate_field.py" --example-csv "$csv"
    echo "接着运行: ./fusion/scripts/run.sh calibrate-csv $csv"
}

case "${1:-help}" in
    help|-h|--help) usage ;;
    build) build ;;
    demo) shift; run_demo "${1:-15}" ;;
    fusion) run_fusion ;;
    all) run_all ;;
    bridge) run_bridge ;;
    rviz-calib) rviz_calib ;;
    calibrate-live) calibrate_live ;;
    calibrate-csv) shift; calibrate_csv "$@" ;;
    calibrate-manual) shift; calibrate_manual "$@" ;;
    calibrate-example) shift; calibrate_example "$@" ;;
    camera) shift; run_camera "$@" ;;
    *)
        echo "未知命令: $1" >&2
        usage >&2
        exit 2
        ;;
esac
