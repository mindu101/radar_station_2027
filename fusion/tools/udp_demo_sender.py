#!/usr/bin/env python3
"""没有相机/激光时，用合成数据测试 fusion_udp_listener 的 UDP 链路。

  ./build/fusion_udp_listener --port 11411 &
  python3 fusion/tools/udp_demo_sender.py --port 11411 --seconds 20

发送内容：
  L ...  三个目标，10Hz，场地坐标系（默认不做外参变换）
  C ...  同样三个目标，30Hz，带 0.1m 噪声
"""

from __future__ import annotations

import argparse
import math
import random
import socket
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=11411)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--noise", type=float, default=0.1)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.host, args.port)
    robots = [
        # label, center_x, center_y, radius, omega, phase
        (101, 14.0, 7.5, 4.0, 0.30, 0.0),
        (102, 13.0, 7.0, 3.0, 0.24, 1.5),
        (103, 15.0, 8.0, 3.5, 0.36, 3.0),
    ]

    def position(robot, t):
        _, cx, cy, radius, omega, phase = robot
        angle = omega * t + phase
        return cx + radius * math.cos(angle), cy + radius * math.sin(angle)

    started = time.monotonic()
    next_camera = started
    next_lidar = started
    try:
        while time.monotonic() - started < args.seconds:
            now = time.monotonic()
            t = now - started
            if now >= next_lidar:
                lines = []
                for index, robot in enumerate(robots, start=1):
                    x, y = position(robot, t)
                    # 这里假设激光已经和场地同坐标系；真实部署用 --calib 做变换。
                    lines.append(f"L {index} {x:.6f} {y:.6f} 0 0 0 0 0")
                sock.sendto("\n".join(lines).encode("ascii"), destination)
                next_lidar += 0.1
            if now >= next_camera:
                lines = []
                for robot in robots:
                    x, y = position(robot, t)
                    lines.append(
                        "C {label} {x:.6f} {y:.6f} 0.95".format(
                            label=robot[0],
                            x=x + random.gauss(0.0, args.noise),
                            y=y + random.gauss(0.0, args.noise),
                        )
                    )
                sock.sendto("\n".join(lines).encode("ascii"), destination)
                next_camera += 1.0 / 30.0
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
