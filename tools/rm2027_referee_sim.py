#!/usr/bin/env python3
"""PTY referee-system simulator for the 2027 radar messenger.

Run this script, pass the printed PTY path to the radar process, and it will
send 0x0001/0x020C/0x020E plus 0x0A01~0x0A06 frames while printing valid frames
received from the application.  It uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import os
import pty
import select
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from rm2027_protocol import build, parse


def send(master: int, cmd: int, payload: bytes, seq: int) -> int:
    frame = build(cmd, payload, seq)
    os.write(master, frame)
    return (seq + 1) & 0xFF


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--robot-id", type=int, default=109,
                        help="blue radar ID (red radar is 9)")
    args = parser.parse_args()
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    os.set_blocking(master, False)
    print(f"referee PTY: {slave_name}", flush=True)
    print("Start the radar process with this device, then press Enter here.", flush=True)
    try:
        input()
    except EOFError:
        pass
    seq = 0
    started = time.monotonic()
    next_tick = started
    rx = bytearray()
    while time.monotonic() - started < args.seconds:
        now = time.monotonic()
        if now >= next_tick:
            # game_type=1 (RMUC), game_progress=4 (battle), 400 seconds left
            seq = send(master, 0x0001, struct.pack("<BHQ", 0x41, 400, int(now * 1e6)), seq)
            # 0x0201 tells the radar its own robot ID.  messenger.poll() waits
            # for this packet before it starts sending position and command data.
            # The payload is the 2027 robot_status_t layout (17 bytes).
            seq = send(master, 0x0201,
                       struct.pack("<BBHHHHHfB", args.robot_id, 1,
                                   10000, 10000, 100, 1000, 80, 28.0, 7), seq)
            seq = send(master, 0x020C, struct.pack("<H", 100), seq)
            # level 1, key change allowed, no active armor fortification
            seq = send(master, 0x020E, bytes((0x28, 0x00)) + struct.pack("<HHH", 0, 0, 0), seq)
            # positions for opponent 1,3,4,7; reserved pairs are zero
            seq = send(master, 0x0A01, struct.pack("<12H", 500, 300, 0, 0, 700, 400, 900, 600, 0, 0, 1200, 800), seq)
            seq = send(master, 0x0A02, struct.pack("<6H", 2000, 0, 1500, 1200, 0, 5000), seq)
            seq = send(master, 0x0A03, struct.pack("<5H", 100, 80, 70, 0, 200), seq)
            seq = send(master, 0x0A04, struct.pack("<HHII", 20, 100, 0, 0), seq)
            seq = send(master, 0x0A05, bytes(47), seq)
            seq = send(master, 0x0A06, b"A1b2C3", seq)
            next_tick += 1.0
        ready, _, _ = select.select([master], [], [], 0.05)
        if master in ready:
            try:
                rx.extend(os.read(master, 4096))
            except BlockingIOError:
                pass
            while len(rx) >= 9:
                if rx[0] != 0xA5:
                    del rx[0]
                    continue
                length = struct.unpack_from("<H", rx, 1)[0]
                total = 9 + length
                if len(rx) < total:
                    break
                frame = bytes(rx[:total])
                del rx[:total]
                try:
                    cmd, frame_seq, payload = parse(frame)
                    print(f"TX cmd=0x{cmd:04x} seq={frame_seq} len={len(payload)} data={payload.hex(' ')}", flush=True)
                except ValueError as error:
                    print(f"invalid outgoing frame: {error}", file=sys.stderr, flush=True)
    os.close(master)
    os.close(slave)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
