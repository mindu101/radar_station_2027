#!/usr/bin/env bash
# 上一届相机程序（复制到本仓库）的便捷入口。
#
#   ./camera_app/run.sh build        构建（含 fusion_camera）
#   ./camera_app/run.sh probe        列出 Aravis 设备
#   ./camera_app/run.sh calibrate [原有标定程序参数...]
#   ./camera_app/run.sh run [原有 radar 参数...]
#   ./camera_app/run.sh fusion [原有 radar 参数...]   # 启动激光桥 + 融合
#
# 海康 MV-CA016-10UC 默认序列号 00J17982005，可用 HIK_SERIAL 覆盖。
# 标定直接调用本目录里复制并编译的“原有标定程序”，不重新实现相机标定。

set -euo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$APP_DIR"

SERIAL="${HIK_SERIAL:-00J17982005}"
RADAR="$APP_DIR/build/bin/radar"
CALIBRATION="$APP_DIR/build/bin/calibration"

usage() {
    cat <<'EOF'
用法: ./camera_app/run.sh <命令>

  build        构建（含 fusion_camera）
  probe        列出 Aravis 设备
  calibrate   调用本目录原有的标定程序
  run          运行相机程序
  fusion       启动激光桥 + 相机融合

环境变量: HIK_SERIAL（默认 00J17982005）、FUSION_*、FUSION_START_BRIDGE
EOF
}

build() {
    cmake -S "$APP_DIR" -B "$APP_DIR/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$APP_DIR/build" -j"$(nproc 2>/dev/null || echo 2)"
}

default_radar_args() {
    if [[ $# -eq 0 ]]; then
        set -- -V=-2 -m=15 -s=/dev/ttyUSB0 "$SERIAL"
    fi
    printf '%s\0' "$@"
}

case "${1:-help}" in
    help|-h|--help)
        usage
        ;;
    build)
        build
        ;;
    probe)
        exec arv-tool-0.8
        ;;
    calibrate)
        shift
        if [[ ! -x "$CALIBRATION" ]]; then
            echo "未找到 $CALIBRATION，先运行 ./camera_app/run.sh build" >&2
            exit 1
        fi
        if [[ " $* " != *" -V="* && " $* " != *" --V="* ]]; then
            set -- -V=aravis "$@"
        fi
        exec "$CALIBRATION" "$@"
        ;;
    run)
        shift
        if [[ ! -x "$RADAR" ]]; then
            echo "未找到 $RADAR，先运行 ./camera_app/run.sh build" >&2
            exit 1
        fi
        mapfile -d '' -t args < <(default_radar_args "$@")
        exec "$RADAR" "${args[@]}"
        ;;
    fusion)
        shift
        if [[ ! -x "$RADAR" ]]; then
            echo "未找到 $RADAR，先运行 ./camera_app/run.sh build" >&2
            exit 1
        fi
        mapfile -d '' -t args < <(default_radar_args "$@")
        FUSION_CAMERA_BIN="$RADAR" "$APP_DIR/../fusion/scripts/run.sh" camera "${args[@]}"
        ;;
    *)
        echo "未知命令: $1" >&2
        usage >&2
        exit 2
        ;;
esac
