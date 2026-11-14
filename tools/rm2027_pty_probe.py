#!/usr/bin/env python3
"""Minimal client for rm2027_referee_sim.py.

This is useful when the full radar application is not built yet: it opens the
PTY as if it were /dev/ttyUSB0, prints received frames, and sends one valid
0x0301/0x0121 radar command back to the simulator.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time
import tty

sys.path.insert(0, os.path.dirname(__file__))
from rm2027_protocol import build, parse


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("device", help="PTY path printed by rm2027_referee_sim.py")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    try:
        # data_cmd_id=0x0121, sender=109 (blue radar), receiver=0x8080.
        radar_command = bytes(13)
        payload = struct.pack("<HHH", 0x0121, 109, 0x8080) + radar_command
        os.write(fd, build(0x0301, payload, 0))

        started = time.monotonic()
        received = bytearray()
        while time.monotonic() - started < args.seconds:
            ready, _, _ = select.select([fd], [], [], 0.2)
            if fd not in ready:
                continue
            received.extend(os.read(fd, 4096))
            while len(received) >= 9:
                if received[0] != 0xA5:
                    del received[0]
                    continue
                length = struct.unpack_from("<H", received, 1)[0]
                total = 9 + length
                if len(received) < total:
                    break
                frame = bytes(received[:total])
                del received[:total]
                try:
                    cmd, seq, data = parse(frame)
                    print(f"RX cmd=0x{cmd:04x} seq={seq} len={len(data)} data={data.hex(' ')}")
                except ValueError as error:
                    print(f"invalid frame: {error}", file=sys.stderr)
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
