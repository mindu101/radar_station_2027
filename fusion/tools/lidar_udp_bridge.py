#!/usr/bin/env python3
"""把 ROS1 的 /learning/tracks 转发成 fusion UDP 文本协议。

在现有雷达 docker（不改 radar_docker 仓库）里运行，例如：

  docker run --rm --network host \
    -e ROS_MASTER_URI=http://127.0.0.1:11321 -e ROS_IP=127.0.0.1 \
    -v /home/mindu/radar/radar_docker/dev_ws:/work/dev_ws:ro \
    -v /home/mindu/radar_station_2027/fusion:/fusion:ro \
    radar-et3:noetic bash -lc '
      source /opt/ros/noetic/setup.bash &&
      source /work/dev_ws/devel/setup.bash &&
      python3 /fusion/tools/lidar_udp_bridge.py --port 11411'

每一帧 TargetStateArray 打包成一个 UDP datagram，每行一条轨迹：
  L <id> <x> <y> <z> <vx> <vy> <vz> <predicted>
"""

from __future__ import annotations

import argparse
import socket
import sys

import rospy
from radar_learning.msg import TargetStateArray


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/learning/tracks")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=11411)
    parser.add_argument("--queue-size", type=int, default=1)
    args = parser.parse_args()
    if not (0 < args.port < 65536):
        parser.error("--port 必须在 1..65535")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.host, args.port)
    sent_frames = 0

    def callback(message: TargetStateArray) -> None:
        nonlocal sent_frames
        lines = []
        for target in message.targets:
            lines.append(
                "L {id} {x:.6f} {y:.6f} {z:.6f} {vx:.6f} {vy:.6f} {vz:.6f} {pred}".format(
                    id=target.id,
                    x=target.position.x,
                    y=target.position.y,
                    z=target.position.z,
                    vx=target.velocity.x,
                    vy=target.velocity.y,
                    vz=target.velocity.z,
                    pred=1 if target.predicted else 0,
                )
            )
        if not lines:
            return
        sock.sendto("\n".join(lines).encode("ascii"), destination)
        sent_frames += 1

    rospy.init_node("lidar_udp_bridge", anonymous=True)
    rospy.Subscriber(args.topic, TargetStateArray, callback, queue_size=args.queue_size)
    rospy.loginfo("forwarding %s to udp://%s:%d", args.topic, args.host, args.port)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    rospy.loginfo("stopped after %d frames", sent_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
