#!/usr/bin/env python3
"""Small 2027 referee-protocol helper for offline/PTY tests.

Examples:
  ./tools/rm2027_protocol.py build --cmd 0x020e --payload-hex 0101000000000000
  ./tools/rm2027_protocol.py parse --hex a50b000001...
"""

from __future__ import annotations

import argparse
import struct
import sys

SOF = 0xA5
MAX_DATA = 118


def crc8(data: bytes) -> int:
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x31) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8408 if value & 1 else value >> 1
    return value & 0xFFFF


def build(cmd: int, payload: bytes, seq: int = 0) -> bytes:
    if len(payload) > MAX_DATA:
        raise ValueError("payload exceeds 118 bytes")
    header = bytearray(struct.pack("<BHB", SOF, len(payload), seq))
    header.append(crc8(header))
    body = bytes(header) + struct.pack("<H", cmd) + payload
    return body + struct.pack("<H", crc16(body))


def parse(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) < 9 or frame[0] != SOF:
        raise ValueError("frame too short or bad SOF")
    if crc8(frame[:4]) != frame[4]:
        raise ValueError("bad CRC8")
    length = struct.unpack_from("<H", frame, 1)[0]
    expected = 5 + 2 + length + 2
    if length > MAX_DATA or len(frame) != expected:
        raise ValueError("invalid data_length")
    if crc16(frame[:-2]) != struct.unpack_from("<H", frame, len(frame) - 2)[0]:
        raise ValueError("bad CRC16")
    return struct.unpack_from("<H", frame, 5)[0], frame[3], frame[7:-2]


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="op", required=True)
    build_parser = sub.add_parser("build")
    build_parser.add_argument("--cmd", type=lambda value: int(value, 0), required=True)
    build_parser.add_argument("--payload-hex", default="")
    build_parser.add_argument("--seq", type=int, default=0)
    build_parser.add_argument("--raw", action="store_true", help="write binary instead of hex")
    parse_parser = sub.add_parser("parse")
    parse_parser.add_argument("--hex", help="frame as hexadecimal; defaults to stdin")
    args = parser.parse_args()
    if args.op == "build":
        frame = build(args.cmd, bytes.fromhex(args.payload_hex), args.seq)
        if args.raw:
            sys.stdout.buffer.write(frame)
        else:
            print(frame.hex(" "))
        return 0
    raw = bytes.fromhex(args.hex) if args.hex else bytes.fromhex(sys.stdin.read())
    cmd, seq, payload = parse(raw)
    print(f"cmd=0x{cmd:04x} seq={seq} data_length={len(payload)} payload={payload.hex(' ')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
