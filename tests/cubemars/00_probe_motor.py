#!/usr/bin/env python3
"""tests/cubemars/00_probe_motor.py

ACTIVE but SAFE probe of the CubeMars AK60-6 V3.0.

Sends a single MIT frame with **all-zero parameters** (Kp=0, Kd=0,
position=0, velocity=0, torque_ff=0).  This commands the motor to apply
**zero torque** -- the shaft will NOT move -- but in V3.0 firmware the
motor still issues one feedback frame in reply (whether configured for
Periodic or Query-Reply mode).  Therefore this is a clean ping:

    PASS  → motor is alive, wiring is correct, bitrate is right.
    FAIL  → genuine bus / wiring / power / bitrate problem.

Frame format (V3.0, manual §4.2)::

    arbitration ID = (CAN_PACKET_SET_MIT << 8) | ESC_ID = (8 << 8) | 0x02 = 0x0802
    payload (Kp first byte order)  = all zeros (after float→uint encoding)

Run:
    sudo python3 tests/cubemars/00_probe_motor.py              # gs_usb (DSD TECH / Waveshare USB)
    sudo python3 tests/cubemars/00_probe_motor.py --socketcan    # built-in mttcan via can0
"""
from __future__ import annotations

import argparse
import sys
import time
import can

BITRATE         = 1_000_000
DEFAULT_ESC_ID  = 0x02   # confirmed via active probe on this bench
MIT_PACKET_TYPE = 8
FB_PACKET_TYPE  = 0x29

# A safe payload: encode (p=0, v=0, kp=0, kd=0, tau=0) for AK60-6 limits
# p_max=±12.56 rad, v_max=±60 rad/s, t_max=±12 N·m, kp_max=500, kd_max=5.
# For each signed range mid is 0 → midpoint of the uint range = 0x800 / 0x80000.
# For kp,kd (zero offset, 0..max) → 0.
#   p_int  (16-bit, mid)  = 0x8000
#   v_int  (12-bit, mid)  = 0x800
#   kp_int (12-bit, zero) = 0x000
#   kd_int (12-bit, zero) = 0x000
#   t_int  (12-bit, mid)  = 0x800
# Layout (Kp-first):
#   byte0 = kp_int >> 4                                = 0x00
#   byte1 = (kp_int & 0xF) << 4 | (kd_int >> 8)        = 0x00
#   byte2 = kd_int & 0xFF                              = 0x00
#   byte3 = p_int >> 8                                 = 0x80
#   byte4 = p_int & 0xFF                               = 0x00
#   byte5 = v_int >> 4                                 = 0x80
#   byte6 = (v_int & 0xF) << 4 | (t_int >> 8)          = 0x08
#   byte7 = t_int & 0xFF                               = 0x00
SAFE_ZERO_MIT = bytes([0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x08, 0x00])

REPLIES_TO_WAIT_FOR = 3
WAIT_PER_REPLY      = 0.5    # seconds
POLL_CHUNK          = 0.05


def decode_feedback(data: bytes) -> str:
    if len(data) < 8:
        return f"(short payload, {len(data)} bytes)"
    pos_int = int.from_bytes(data[0:2], "big", signed=True)
    spd_int = int.from_bytes(data[2:4], "big", signed=True)
    cur_int = int.from_bytes(data[4:6], "big", signed=True)
    temp_c  = int.from_bytes(data[6:7], "big", signed=True)
    err     = data[7]
    return (
        f"pos={pos_int * 0.1:+8.2f} deg   "
        f"spd={spd_int * 10:+8d} ERPM   "
        f"I={cur_int * 0.01:+5.2f} A   "
        f"T={temp_c:3d} C   err={err}"
    )


def wait_for_reply(bus: can.BusABC, timeout: float):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=min(POLL_CHUNK, remaining))
        if msg is None:
            continue
        return msg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--socketcan", action="store_true",
                    help="Use SocketCAN can0 instead of gs_usb (for SN65HVD230 / mttcan)")
    ap.add_argument("--channel", default="can0",
                    help="SocketCAN channel name (default: can0)")
    ap.add_argument("--id", type=lambda x: int(x, 0), default=DEFAULT_ESC_ID,
                    dest="esc_id",
                    help="Motor ESC_ID in decimal or hex (default: 0x02 = 2)")
    args = ap.parse_args()

    esc_id = args.esc_id
    tx_id  = (MIT_PACKET_TYPE << 8) | esc_id
    rx_id  = (FB_PACKET_TYPE  << 8) | esc_id

    backend = "socketcan" if args.socketcan else "gs_usb"
    print("CubeMars AK60-6 active probe (safe zero-torque MIT)")
    print(f"  backend     : {backend}{'  channel=' + args.channel if args.socketcan else ''}")
    print(f"  bitrate     : {BITRATE // 1000} kbps")
    print(f"  TX ID       : 0x{tx_id:04X}  (extended, MIT to ESC 0x{esc_id:02X})")
    print(f"  TX payload  : {SAFE_ZERO_MIT.hex().upper()}  (all zeros, no torque)")
    print(f"  expected RX : 0x{rx_id:04X}  (extended feedback from ESC 0x{esc_id:02X})")
    print()

    try:
        if args.socketcan:
            bus = can.Bus(channel=args.channel, interface="socketcan")
        else:
            bus = can.Bus(interface="gs_usb", channel=0, bitrate=BITRATE, index=0)
    except Exception as exc:
        print(f"FAIL: Could not open {backend} bus: {exc}")
        if args.socketcan:
            print(f"      → Is can0 up?  Run: sudo ip link set {args.channel} up type can bitrate 1000000")
        else:
            print("      → Is the USB-to-CAN adapter plugged in?  Run as sudo?")
        return 1

    tx = can.Message(arbitration_id=tx_id, data=SAFE_ZERO_MIT, is_extended_id=True)

    print("  → Sending zero-MIT command ...", end="  ", flush=True)
    try:
        bus.send(tx)
        print("OK")
    except can.CanError as exc:
        print(f"FAIL\n  Send error: {exc}")
        bus.shutdown()
        return 1

    print(f"  ← Waiting up to {WAIT_PER_REPLY * REPLIES_TO_WAIT_FOR:.1f} s "
          f"for feedback from 0x{rx_id:04X} ...")
    print()

    got_any = False
    feedback_seen = 0
    for i in range(REPLIES_TO_WAIT_FOR):
        msg = wait_for_reply(bus, WAIT_PER_REPLY)
        if msg is None:
            break
        got_any = True
        is_fb = msg.is_extended_id and msg.arbitration_id == rx_id
        tag   = "  ← FEEDBACK" if is_fb else "  (other frame)"
        print(
            f"    rx[{i}]  ID=0x{msg.arbitration_id:08X}  "
            f"ext={'Y' if msg.is_extended_id else 'N'}  "
            f"dlc={msg.dlc}  data={bytes(msg.data).hex().upper()}{tag}"
        )
        if is_fb:
            feedback_seen += 1
            print(f"           {decode_feedback(bytes(msg.data))}")

    bus.shutdown()

    print()
    if feedback_seen > 0:
        print(f"PASS — motor replied with {feedback_seen} feedback frame(s).")
        print(f"       Motor is alive and ESC_ID 0x{esc_id:02X} is correct.")
        return 0

    if got_any:
        print(f"PARTIAL — bus traffic seen but no frame from ESC 0x{esc_id:02X}.")
        print(f"  → Motor's ESC_ID might be different from 0x{esc_id:02X}.")
        print(f"    Re-check R-Link: Controller ID = {esc_id} (0x{esc_id:02X}).")
        return 2

    print("FAIL — no reply at all.  Try these in order:")
    print("  1. Re-seat CAN_H/CAN_L (try swapping them).")
    print("  2. Verify motor power: status LED on the driver board lit?")
    print("  3. Open R-Link via UART (not CAN): confirm the motor is alive")
    print("     and read its current CAN bitrate and ESC_ID.")
    print("  4. Termination: add a 120 Ω resistor across CAN_H/CAN_L at")
    print("     the Jetson end (SN65HVD230 has no built-in terminator).")
    print("     Motor end usually has 120 Ω onboard already.")
    return 3


if __name__ == "__main__":
    sys.exit(main())
