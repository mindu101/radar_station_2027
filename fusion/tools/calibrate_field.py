#!/usr/bin/env python3
"""激光雷达 -> 场地坐标系 外参标定。

输出的 JSON 可以直接给：
  fusion_udp_listener --calib calib.json
或者 C++ 里的 fusion::FieldTransform::load_json()。

两种模式
--------
1) 离线 CSV：

   role,id,lx,ly,lz,fx,fy,fz
   fit,corner_a,1.20,-0.50,0.00,0.00,0.00,0.00
   fit,corner_b,4.90,-0.48,0.00,3.70,0.00,0.00
   ...
   check,post_1,3.10,2.20,0.00,1.90,2.70,0.00

   role = fit（参与求解）或 check（只验证）。至少 4 个 fit、2 个 check。
   lx/ly/lz 是雷达点云里的坐标（ws_scan，米）；fx/fy/fz 是该点在场地系的
   坐标（米）。场地系与 0x0305 一致：红方补给区角落为原点，X 向蓝方，
   Y 向红方停机坪。

   python3 fusion/tools/calibrate_field.py \
       --csv fusion/tmp/points.csv \
       --output fusion/tmp/field_calibration.json

2) 在线点击（在雷达 ROS 容器里跑，RViz 用 Publish Point 点已知标识点）：

   docker run --rm -it --network host \
     -e ROS_MASTER_URI=http://127.0.0.1:11321 -e ROS_IP=127.0.0.1 \
     -v /home/mindu/radar_station_2027/fusion:/fusion:ro \
     -v /home/mindu/radar_station_2027/fusion/tmp:/data \
     radar-et3:noetic bash -lc '
       source /opt/ros/noetic/setup.bash &&
       python3 /fusion/tools/calibrate_field.py --live --output /data/field_calibration.json'

   每次点击后按提示输入该点在场地系的 x y z（米），可选第三个参数
   fit/check（默认 fit）。直接回车结束采集。

自检（不需要 ROS/硬件）：

   python3 fusion/tools/calibrate_field.py --self-test
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


def solve_transform(lidar: np.ndarray, field: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Kabsch/SVD 求 field = R * lidar + t。"""
    lidar_mean = lidar.mean(axis=0)
    field_mean = field.mean(axis=0)
    a = lidar - lidar_mean
    b = field - field_mean
    u, singular, vt = np.linalg.svd(a.T @ b)
    if singular[0] < 1e-9 or singular[1] / singular[0] < 1e-3:
        raise ValueError("标定点重合或近似共线，无法求解")
    correction = np.eye(3)
    correction[2, 2] = math.copysign(1.0, np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ correction @ u.T
    translation = field_mean - rotation @ lidar_mean
    return rotation, translation


def parse_float(text: str, name: str) -> float:
    try:
        value = float(text)
    except ValueError as error:
        raise ValueError(f"{name} 不是数字: {text!r}") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} 不是有限数: {text!r}")
    return value


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"role", "id", "lx", "ly", "lz", "fx", "fy", "fz"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"CSV 需要列: {', '.join(sorted(required))}")
        rows = []
        for line_number, row in enumerate(reader, start=2):
            role = row["role"].strip().lower()
            if role not in ("fit", "check"):
                raise ValueError(f"第 {line_number} 行 role 必须是 fit 或 check")
            try:
                rows.append(
                    {
                        "role": role,
                        "id": row["id"].strip(),
                        "lidar": [parse_float(row[k], k) for k in ("lx", "ly", "lz")],
                        "field": [parse_float(row[k], k) for k in ("fx", "fy", "fz")],
                    }
                )
            except ValueError as error:
                raise ValueError(f"第 {line_number} 行: {error}") from error
    return rows


def collect_live(topic: str) -> list[dict]:
    import queue

    import rospy
    from geometry_msgs.msg import PointStamped

    rospy.init_node("fusion_field_calibration", anonymous=True)
    clicked: "queue.Queue[PointStamped]" = queue.Queue()
    rospy.Subscriber(topic, PointStamped, clicked.put, queue_size=10)
    print("在 RViz 里用 Publish Point 点击已知场地标识点；每次点击后输入它的场地坐标。")
    print("场地系与 0x0305 一致：红方补给区角落为原点，X 向蓝方，Y 向红方停机坪，单位米。")
    print("至少 4 个 fit + 2 个 check。直接回车结束。\n")
    rows: list[dict] = []
    while not rospy.is_shutdown():
        try:
            message = clicked.get(timeout=0.2)
        except queue.Empty:
            continue
        point = message.point
        print(f"点击点 (雷达系): {point.x:.4f} {point.y:.4f} {point.z:.4f}")
        try:
            text = input("场地系 x y z [fit|check]（回车结束）: ").strip()
        except EOFError:
            break
        if not text:
            break
        parts = text.split()
        if len(parts) < 3:
            print("需要至少 3 个数字，忽略\n")
            continue
        try:
            field = [parse_float(parts[i], f"坐标{i + 1}") for i in range(3)]
            role = parts[3].lower() if len(parts) > 3 else "fit"
            if role not in ("fit", "check"):
                raise ValueError("role 只能是 fit 或 check")
        except ValueError as error:
            print(f"{error}，忽略\n")
            continue
        rows.append(
            {
                "role": role,
                "id": f"click_{len(rows) + 1}",
                "lidar": [point.x, point.y, point.z],
                "field": field,
            }
        )
        fit = sum(row["role"] == "fit" for row in rows)
        check = len(rows) - fit
        print(f"已采集 {len(rows)} 个点（fit={fit}, check={check}），再点下一个……\n")
    return rows


def collect_manual() -> list[dict]:
    """直接手输：雷达点云坐标 + 对应场地坐标，不需要 RViz。"""
    print("每行输入: 雷达系 lx ly lz   场地系 fx fy fz   [fit|check]")
    print("例如: 1.23 -0.45 0.00   0.00 0.00 0.00 fit")
    print("逗号或空格分隔都行；直接回车结束。至少 4 个 fit + 2 个 check。\n")
    rows: list[dict] = []
    while True:
        try:
            text = input(f"[{len(rows) + 1}] 回车结束: ").strip()
        except EOFError:
            break
        if not text:
            break
        parts = text.replace(",", " ").split()
        if len(parts) not in (6, 7):
            print("需要 6 个坐标，最后可选 fit/check；忽略\n")
            continue
        try:
            values = [parse_float(parts[i], f"数值{i + 1}") for i in range(6)]
            role = parts[6].lower() if len(parts) == 7 else "fit"
            if role not in ("fit", "check"):
                raise ValueError("角色只能是 fit 或 check")
        except ValueError as error:
            print(f"{error}，忽略\n")
            continue
        rows.append(
            {
                "role": role,
                "id": f"manual_{len(rows) + 1}",
                "lidar": values[:3],
                "field": values[3:6],
            }
        )
        fit = sum(row["role"] == "fit" for row in rows)
        print(f"已记录 {len(rows)} 个点（fit={fit}, check={len(rows) - fit}）\n")
    return rows


def compute(rows: list[dict], max_error: float, force: bool) -> dict:
    if not rows:
        raise ValueError("没有标定点")
    ids = [row["id"] for row in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("标定点 id 有重复")
    fit = [row for row in rows if row["role"] == "fit"]
    check = [row for row in rows if row["role"] == "check"]
    if len(fit) < 4:
        raise ValueError(f"fit 点太少（{len(fit)}），至少需要 4 个")
    if len(check) < 2:
        raise ValueError(f"check 点太少（{len(check)}），至少需要 2 个")

    lidar = np.array([row["lidar"] for row in rows], dtype=float)
    field = np.array([row["field"] for row in rows], dtype=float)
    fit_mask = np.array([row["role"] == "fit" for row in rows])
    rotation, translation = solve_transform(lidar[fit_mask], field[fit_mask])

    errors = np.linalg.norm(lidar @ rotation.T + translation - field, axis=1)
    for row, error in zip(rows, errors):
        print(f"{row['id']:>10s} {row['role']:>5s} error={error:.4f} m")
    for name, mask in (("fit", fit_mask), ("check", ~fit_mask)):
        values = errors[mask]
        print(f"{name}: RMSE={math.sqrt(float(np.mean(values**2))):.4f} m max={float(values.max()):.4f} m")
    if float(errors.max()) > max_error and not force:
        raise ValueError(
            f"最大误差 {float(errors.max()):.4f} m 超过 --max-error {max_error} m；"
            "请检查标定点，或加 --force 强制写入"
        )
    return {
        "rotation": [float(v) for v in rotation.reshape(-1)],
        "translation": [float(v) for v in translation],
        "lidar_frame": "ws_scan",
        "field_frame": "field",
        "field_length": 28.0,
        "field_width": 15.0,
        "fit_rmse_m": math.sqrt(float(np.mean(errors[fit_mask] ** 2))),
        "check_rmse_m": math.sqrt(float(np.mean(errors[~fit_mask] ** 2))),
        "max_error_m": float(errors.max()),
        "points": len(rows),
    }


def self_test() -> int:
    angle = 0.37
    rotation = np.array(
        [
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    translation = np.array([1.7, -2.3, 0.4])
    field = np.array(
        [
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [4.0, 3.0, 0.0],
            [2.0, 1.0, 0.2],
            [3.0, 2.0, 0.5],
        ]
    )
    lidar = (field - translation) @ rotation  # R^T * (field - t)
    solved_rotation, solved_translation = solve_transform(lidar[:4], field[:4])
    predicted = lidar @ solved_rotation.T + solved_translation
    error = float(np.linalg.norm(predicted - field, axis=1).max())
    if error > 1e-9:
        print(f"self-test failed: max error {error}")
        return 1
    print(f"self-test passed (max error {error:.2e})")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="激光雷达 -> 场地外参标定")
    parser.add_argument("--csv", type=Path, help="离线标定 CSV")
    parser.add_argument("--live", action="store_true", help="在 ROS 容器里从 RViz 点击采集")
    parser.add_argument("--manual", action="store_true",
                        help="终端手输：雷达 x y z + 场地 x y z（不需要 RViz）")
    parser.add_argument("--topic", default="/clicked_point", help="RViz Publish Point 话题")
    parser.add_argument("--output", type=Path, help="输出 JSON")
    parser.add_argument("--max-error", type=float, default=0.05, help="最大允许误差（米），默认 0.05")
    parser.add_argument("--force", action="store_true", help="误差超限也写入/覆盖已有文件")
    parser.add_argument("--self-test", action="store_true", help="无硬件自检")
    parser.add_argument("--example-csv", type=Path, help="生成一个 CSV 模板后退出")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    if args.example_csv:
        # 生成一份自洽的示例：已知 R/t，再由场地坐标反算雷达坐标。
        angle = -0.25
        rotation = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        translation = np.array([0.6, 0.4, 0.0])
        field_points = [
            (0.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
            (0.0, 3.0, 0.0),
            (4.0, 3.0, 0.0),
            (2.0, 1.0, 0.2),
            (2.5, 0.7, 1.0),
            (1.5, 2.2, 0.4),
        ]
        roles = ["fit", "fit", "fit", "fit", "check", "check", "check"]
        lines = ["role,id,lx,ly,lz,fx,fy,fz"]
        for index, (point, role) in enumerate(zip(field_points, roles), start=1):
            field = np.array(point)
            lidar = rotation.T @ (field - translation)
            lines.append(
                f"{role},p{index},{lidar[0]:.6f},{lidar[1]:.6f},{lidar[2]:.6f},"
                f"{field[0]:.6f},{field[1]:.6f},{field[2]:.6f}"
            )
        args.example_csv.parent.mkdir(parents=True, exist_ok=True)
        args.example_csv.write_text("\n".join(lines) + "\n")
        print(f"已写入自洽示例: {args.example_csv}")
        return 0

    if not args.output:
        parser.error("需要 --output")
    modes = sum((args.csv is not None, args.live, args.manual))
    if modes != 1:
        parser.error("需要且只能指定 --csv、--live、--manual 之一（或 --self-test / --example-csv）")

    try:
        if args.csv is not None:
            rows = read_csv(args.csv)
        elif args.live:
            rows = collect_live(args.topic)
        else:
            rows = collect_manual()
        config = compute(rows, args.max_error, args.force)
    except (OSError, ValueError) as error:
        print(f"标定失败: {error}", file=sys.stderr)
        return 1

    if args.output.exists() and not args.force:
        print(f"{args.output} 已存在；加 --force 覆盖", file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as stream:
        json.dump(config, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"已写入 {args.output}")
    print("下一步: ./fusion/build/fusion_udp_listener --calib " + str(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
