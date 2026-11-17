#!/usr/bin/env python3
"""
Measure FollowJSM reply timing on the real bus without moving the chair.

Replies to each centred JSM joystick frame with a centred frame (``0000``), exactly what the
JSM itself is sending, so the power module sees no command change. Uses JoystickFollower from
rnet_bridge, so the centred-only and re-arm rules (6.3.8) are the ones the bridge will use.

The script refuses to send anything but ``0000``. Run ``candump -td -L can0`` alongside to
see the on-wire gap between each JSM frame and our reply.

Usage: python3 tools/follow_jsm_probe.py [--iface can0] [--seconds 10] [--nibble 2]
"""

import argparse
from pathlib import Path
import socket
import struct
import sys
import time

try:
    from rnet_bridge.bridge_logic import JoystickFollower, ReplyMeter
except ImportError:  # not sourced: use the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rnet_bridge'))
    from rnet_bridge.bridge_logic import JoystickFollower, ReplyMeter

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
FRAME = struct.Struct('=IB3x8s')
ZERO = bytes([0, 0])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--iface', default='can0')
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--nibble', type=lambda v: int(v, 0), default=2,
                        help='joystick module device nibble (measured: 2)')
    args = parser.parse_args()

    follower = JoystickFollower(args.nibble, tx_enabled=True)
    meter = ReplyMeter(window=0.001)
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
    sock.bind((args.iface,))
    sock.settimeout(0.1)
    reply = FRAME.pack(follower.frame_id | CAN_EFF_FLAG, 2, ZERO)

    print(f'Following {follower.frame_id:08X} on {args.iface} for {args.seconds:.0f} s, '
          'sending 0000 only')
    jsm_frames = deflected = failures = 0
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        try:
            raw = sock.recv(16)
        except socket.timeout:
            continue
        received = time.monotonic()
        can_id, dlc, data = FRAME.unpack(raw)
        if not can_id & CAN_EFF_FLAG or not follower.is_jsm_frame(can_id & CAN_EFF_MASK):
            continue
        jsm_frames += 1
        follower.set_command(0.0, 0.0, now=received)
        out = follower.on_jsm_frame(data[:dlc], now=received)
        if out is None:
            deflected += 1
            continue
        if out[1] != ZERO:              # hard guard: this probe never commands motion
            sys.exit(f'refusing to send non-zero data {out[1].hex()}')
        try:
            sock.send(reply)
        except OSError as err:
            failures += 1
            print(f'send failed: {err}')
            continue
        meter.record(received, time.monotonic())

    print(f'JSM frames: {jsm_frames}  replies: {meter.replies}  '
          f'skipped (joystick moved or re-arming): {deflected}  send failures: {failures}')
    print(f'software delay (receive -> send returned): max {meter.max_delay * 1000:.3f} ms, '
          f'{meter.late} over 1 ms')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
