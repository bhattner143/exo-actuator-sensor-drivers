#!/usr/bin/env python3
"""tests/cubemars/00_sniff_motor.py

PASSIVE CAN sniffer for the CubeMars AK60-6 V3.0.  No frames are sent,
so the motor shaft will NOT move.  This is the safest first contact test.

What it does
------------
Opens the DSD TECH adapter via the python-can ``gs_usb`` backend and
listens for 5 seconds.  For each frame received it prints the
arbitration ID, extended flag, and payload.  If a feedback frame from
ESC_ID 0x68 is seen (extended ID 0x2968), it is decoded into the V3.0
feedback fields (position, ERPM, current, MOSFET temperature, error
code).

Expected results
----------------
* **Motor in Periodic Feedback mode (factory default)**: at least one
  frame per ~20 ms with extended ID 0x2968.
* **Motor in Inquiry/Query-Reply mode**: zero frames (motor only replies
  when commanded).  Re-run after sending a command, or switch the R-Link
  CAN Mode setting.
* **No frames at all + adapter loopback PASSED**: motor power off, CAN
  wires swapped, or the motor's ESC_ID is not 0x68.

Run (must be root for libusb access):
    sudo python3 tests/cubemars/00_sniff_motor.py
"""
from __future__ import annotations

import sys
import time
import can

BITRATE        = 1_000_000
EXPECTED_ESC   = 0x68              # CubeMars AK60-6 (confirmed 12 May 2026)
FEEDBACK_TYPE  = 0x29              # V3.0 feedback packet type
EXPECTED_FB_ID = (FEEDBACK_TYPE << 8) | EXPECTED_ESC   # 0x2968
LISTEN_SEC     = 5.0
POLL_CHUNK     = 0.05              # gs_usb recv() must not be called with 0


def decode_feedback(data: bytes) -> str:
    """Decode an 8-byte V3.0 feedback payload to a printable string."""
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


def main() -> int:
    print("CubeMars AK60-6 passive sniffer")
    print(f"  bitrate         : {BITRATE // 1000} kbps")
    print(f"  expected ESC_ID : 0x{EXPECTED_ESC:02X} ({EXPECTED_ESC})")
    print(f"  expected feedback ID : 0x{EXPECTED_FB_ID:04X} (extended)")
    print(f"  listening for   : {LISTEN_SEC:.1f} s   (NO frames sent)")
    print()

    try:
        bus = can.Bus(interface="gs_usb", channel=0, bitrate=BITRATE, index=0)
    except Exception as exc:
        print(f"FAIL: Could not open gs_usb bus: {exc}")
        print("      → DSD TECH adapter plugged in?  Running with sudo?")
        return 1

    print(f"  bus open: {bus.channel_info!r}")
    print()
    print("  time(s)   ID         ext  dlc  data")
    print("  " + "-" * 72)

    total           = 0
    matched         = 0
    last_feedback   = None
    seen_ids: set[int] = set()

    deadline = time.monotonic() + LISTEN_SEC
    start    = time.monotonic()
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            msg = bus.recv(timeout=min(POLL_CHUNK, remaining))
            if msg is None:
                continue
            total += 1
            seen_ids.add(msg.arbitration_id)
            elapsed = msg.timestamp - start if msg.timestamp else time.monotonic() - start
            tag = ""
            if msg.is_extended_id and msg.arbitration_id == EXPECTED_FB_ID:
                matched += 1
                last_feedback = msg
                tag = "  ← FEEDBACK from ESC 0x68"
            print(
                f"  {elapsed:6.3f}   "
                f"0x{msg.arbitration_id:08X}  "
                f"{'Y' if msg.is_extended_id else 'N'}    "
                f"{msg.dlc:>2}  "
                f"{bytes(msg.data).hex().upper()}"
                f"{tag}"
            )
    finally:
        bus.shutdown()

    print()
    print("  " + "-" * 72)
    print(f"  Total frames     : {total}")
    print(f"  Frames from 0x68 : {matched}")
    print(f"  Unique IDs seen  : {sorted(hex(i) for i in seen_ids)}")

    if last_feedback is not None:
        print()
        print("  Last feedback decoded:")
        print(f"    {decode_feedback(bytes(last_feedback.data))}")
        print()
        print("PASS — motor is alive and broadcasting Periodic Feedback.")
        return 0

    if total > 0:
        print()
        print("PARTIAL — Frames seen on bus, but none from ESC 0x68.")
        print("  → Check motor ESC_ID setting in R-Link (expected 0x68).")
        return 2

    print()
    print("NO TRAFFIC SEEN.  Possible causes (in order of likelihood):")
    print("  1. Motor in Inquiry/Query-Reply mode → switch R-Link CAN Mode")
    print("     to 'Periodic Feedback' and power-cycle the motor.")
    print("  2. Motor power off, or 48 V PSU not connected to XT30 pins 1/2.")
    print("  3. CAN_H and CAN_L swapped at the XT30 plug.")
    print("     Expected: pin 3 (white) = CAN_H, pin 4 (blue) = CAN_L.")
    print("  4. Missing 120 Ω termination on the motor end of the bus.")
    print("  5. Wrong CAN bitrate set in R-Link (must be 1 Mbps).")
    return 3


if __name__ == "__main__":
    sys.exit(main())
