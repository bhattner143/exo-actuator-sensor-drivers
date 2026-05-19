#!/usr/bin/env python3
"""tests/cubemars/00_loopback_dsd_tech.py

Hardware loopback test for the DSD TECH SH-C30A USB-to-CAN adapter.
Uses the python-can ``gs_usb`` backend -- no ``gs_usb`` kernel module needed.

Two modes
---------
1. **Software loopback** (default, no extra wiring):
   Opens the bus with ``receive_own_messages=True``.  The adapter echoes
   every transmitted frame back into the receive queue in firmware.
   Verifies the python-can driver + USB transport but NOT the physical
   CAN_H/CAN_L lines.

2. **Physical loopback** (wire test):
   Short CAN_H → CAN_H and CAN_L → CAN_L with a 120 Ω resistor between
   them, then run with ``--physical``.  Proves the differential output
   wiring is intact.

Run (always as root for USB access):
    sudo python3 tests/cubemars/00_loopback_dsd_tech.py
    sudo python3 tests/cubemars/00_loopback_dsd_tech.py --physical
"""
from __future__ import annotations

import sys
import time
import argparse
import can

BITRATE       = 1_000_000
TEST_ID       = 0x1234567          # 29-bit extended frame
TEST_DATA     = bytes([0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04])
RECV_TIMEOUT  = 2.0                # seconds to wait for echo
POLL_INTERVAL = 0.05               # recv() chunk size (avoids blocking on 0s)


def _open_bus(receive_own: bool) -> can.BusABC:
    """Open the first gs_usb device at 1 Mbps."""
    return can.Bus(
        interface="gs_usb",
        channel=0,
        bitrate=BITRATE,
        index=0,
        receive_own_messages=receive_own,
    )


def _poll(bus: can.BusABC, deadline: float) -> "can.Message | None":
    """Poll the bus until *deadline* (monotonic seconds), return first frame."""
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=min(POLL_INTERVAL, remaining))
        if msg is not None:
            return msg


def run_loopback(physical: bool) -> bool:
    receive_own = not physical
    mode_label  = "physical" if physical else "software (receive_own_messages)"

    print(f"DSD TECH SH-C30A loopback test — mode: {mode_label}")
    print(f"  bitrate : {BITRATE // 1000} kbps")
    print(f"  test ID : 0x{TEST_ID:07X}  (29-bit extended)")
    print(f"  payload : {TEST_DATA.hex().upper()}")
    print()

    try:
        bus = _open_bus(receive_own)
    except Exception as exc:
        print(f"FAIL: Could not open gs_usb bus: {exc}")
        print("      → Is the DSD TECH adapter plugged in?  Run as sudo?")
        return False

    print(f"  Bus open  : {bus.channel_info!r}")

    tx = can.Message(
        arbitration_id=TEST_ID,
        data=TEST_DATA,
        is_extended_id=True,
    )

    print("  Sending ...", end="  ", flush=True)
    try:
        bus.send(tx)
        print("OK")
    except can.CanError as exc:
        print(f"FAIL\n  Send error: {exc}")
        bus.shutdown()
        return False

    print(f"  Waiting up to {RECV_TIMEOUT} s for echo ...", end="  ", flush=True)
    rx = _poll(bus, time.monotonic() + RECV_TIMEOUT)
    bus.shutdown()

    if rx is None:
        print("FAIL")
        print()
        if physical:
            print("  → No frame received.  Check wiring:")
            print("    • CAN_H  ──[120 Ω]── CAN_L  (short across the adapter terminal)")
            print("    • Ensure both CAN_H and CAN_L are connected to the resistor.")
        else:
            print("  → No echo from firmware.  The adapter may not support")
            print("    receive_own_messages.  Try the physical loopback instead:")
            print("    sudo python3 tests/cubemars/00_loopback_dsd_tech.py --physical")
        return False

    print("GOT FRAME")
    print(f"  Received  : ID=0x{rx.arbitration_id:07X}  "
          f"data={bytes(rx.data).hex().upper()}  "
          f"is_ext={rx.is_extended_id}")

    id_ok   = rx.arbitration_id == TEST_ID
    data_ok = bytes(rx.data) == TEST_DATA
    ext_ok  = rx.is_extended_id

    print()
    print(f"  Arbitration ID : {'OK' if id_ok   else 'MISMATCH'}")
    print(f"  Data payload   : {'OK' if data_ok  else 'MISMATCH'}")
    print(f"  Extended frame : {'OK' if ext_ok   else 'WRONG (got standard)'}")

    if id_ok and data_ok and ext_ok:
        print("\nPASS")
        return True
    else:
        print("\nFAIL: Frame content mismatch.")
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description="DSD TECH CAN adapter loopback test")
    parser.add_argument(
        "--physical",
        action="store_true",
        help="Physical loopback mode (requires 120 Ω resistor across CAN_H/CAN_L)",
    )
    args = parser.parse_args()

    ok = run_loopback(physical=args.physical)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
