#!/usr/bin/env python3
"""tests/cubemars/00_scan_bitrate.py

Sweep through common CAN bitrates and listen passively at each one.
If the motor's bitrate was changed away from 1 Mbps (factory default),
this is the fastest way to find out which rate it is actually on.

Strategy
--------
For each bitrate in [1 M, 500 k, 250 k, 125 k]:
  1. Open the adapter at that bitrate.
  2. Listen 2 s for ANY frame on the bus.
  3. Report frame count and the unique IDs seen.

NO frames are transmitted, so the motor cannot move.

Run as root:
    sudo python3 tests/cubemars/00_scan_bitrate.py
"""
from __future__ import annotations

import sys
import time
import can

BITRATES   = [1_000_000, 500_000, 250_000, 125_000]
LISTEN_SEC = 2.0
POLL_CHUNK = 0.05


def listen(bitrate: int) -> tuple[int, set[int]]:
    try:
        bus = can.Bus(interface="gs_usb", channel=0, bitrate=bitrate, index=0)
    except Exception as exc:
        print(f"  could not open at {bitrate}: {exc}")
        return 0, set()

    count = 0
    ids: set[int] = set()
    deadline = time.monotonic() + LISTEN_SEC
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            msg = bus.recv(timeout=min(POLL_CHUNK, remaining))
            if msg is None:
                continue
            count += 1
            ids.add(msg.arbitration_id)
    finally:
        bus.shutdown()
    return count, ids


def main() -> int:
    print("CubeMars / DSD TECH bitrate sweep")
    print(f"  listening {LISTEN_SEC:.1f} s at each rate, NO frames sent")
    print()

    best_rate  = None
    best_count = 0
    for br in BITRATES:
        print(f"  {br//1000:>4d} kbps ... ", end="", flush=True)
        count, ids = listen(br)
        id_list = ", ".join(f"0x{i:X}" for i in sorted(ids)[:8])
        more    = "" if len(ids) <= 8 else f" (+{len(ids)-8} more)"
        print(f"{count:>4d} frames   unique IDs: [{id_list}{more}]")
        if count > best_count:
            best_count = count
            best_rate  = br

    print()
    if best_rate is None or best_count == 0:
        print("NO TRAFFIC at any bitrate.  This is wiring or power, not bitrate.")
        print("  Likely cause: CAN_H/CAN_L swapped, or motor power off.")
        print("  Next step:  unplug CAN, swap H↔L at the XT30, replug, rerun.")
        return 1

    print(f"BEST MATCH: {best_rate//1000} kbps  ({best_count} frames seen)")
    if best_rate != 1_000_000:
        print(f"  → Motor is NOT at 1 Mbps.  Update R-Link CAN bitrate, or")
        print(f"     change ``BITRATE = {best_rate}`` in your scripts.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
