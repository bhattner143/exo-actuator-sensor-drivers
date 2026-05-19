#!/usr/bin/env python3
"""tests/cubemars/00_loopback_socketcan.py

Loopback test for the Waveshare SN65HVD230 CAN transceiver connected to
the Jetson Orin Nano's built-in CAN controller (``can0``).

Two modes
---------
1. **Software loopback** (default, no extra wiring):
   Brings up the interface with the kernel ``loopback on`` flag.  Transmitted
   frames are reflected back by the kernel without going through the
   SN65HVD230 differential lines.  Verifies the kernel CAN driver +
   python-can stack.

2. **Physical loopback** (wire test):
   Brings up the interface normally (loopback off).  Short CAN_H → CAN_H and
   CAN_L → CAN_L on the SN65HVD230 screw terminal with a 120 Ω resistor
   across them, then run with ``--physical``.  Proves the SN65HVD230
   transceiver and differential wiring are intact.

Run (always as root for CAN interface control):
    sudo python3 tests/cubemars/00_loopback_socketcan.py
    sudo python3 tests/cubemars/00_loopback_socketcan.py --physical
    sudo python3 tests/cubemars/00_loopback_socketcan.py --interface can1
"""
from __future__ import annotations

import sys
import time
import argparse
import subprocess
import can

BITRATE       = 1_000_000
TEST_ID       = 0x1234567          # 29-bit extended frame
TEST_DATA     = bytes([0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04])
RECV_TIMEOUT  = 2.0                # seconds to wait for echo
POLL_INTERVAL = 0.05               # recv() chunk size


def _bring_up(interface: str, loopback: bool) -> None:
    """Bring up the SocketCAN interface, resetting it first if needed."""
    # Silently bring it down first so we can change parameters
    subprocess.run(["ip", "link", "set", interface, "down"],
                   check=False, capture_output=True)

    # Always specify loopback explicitly so a prior "loopback on" doesn't
    # remain sticky on the interface.
    cmd = ["ip", "link", "set", interface, "type", "can",
           "bitrate", str(BITRATE),
           "loopback", "on" if loopback else "off"]
    subprocess.run(cmd, check=True)

    subprocess.run(["ip", "link", "set", interface, "up"], check=True)


def _bring_down(interface: str) -> None:
    subprocess.run(["ip", "link", "set", interface, "down"],
                   check=False, capture_output=True)


def _poll(bus: can.BusABC, deadline: float) -> "can.Message | None":
    """Poll the bus until *deadline* (monotonic seconds), return first frame."""
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=min(POLL_INTERVAL, remaining))
        if msg is not None:
            return msg


def run_loopback(interface: str, physical: bool) -> bool:
    loopback    = not physical
    mode_label  = "physical (loopback off)" if physical else "software (kernel loopback on)"

    print(f"Waveshare SN65HVD230 / SocketCAN loopback test")
    print(f"  interface : {interface}")
    print(f"  mode      : {mode_label}")
    print(f"  bitrate   : {BITRATE // 1000} kbps")
    print(f"  test ID   : 0x{TEST_ID:07X}  (29-bit extended)")
    print(f"  payload   : {TEST_DATA.hex().upper()}")
    print()

    # --- Bring up interface --------------------------------------------------
    print(f"  Bringing up {interface} ...", end="  ", flush=True)
    try:
        _bring_up(interface, loopback)
        print("OK")
    except subprocess.CalledProcessError as exc:
        print(f"FAIL\n  {exc}")
        print(f"  → Is {interface} a valid CAN interface?  Run as sudo?")
        return False

    # --- Open python-can bus -------------------------------------------------
    print(f"  Opening python-can socketcan bus ...", end="  ", flush=True)
    try:
        bus = can.Bus(
            interface="socketcan",
            channel=interface,
            receive_own_messages=loopback,  # needed for software loopback
        )
        print(f"OK  ({bus.channel_info!r})")
    except Exception as exc:
        print(f"FAIL\n  {exc}")
        _bring_down(interface)
        return False

    # --- Send ----------------------------------------------------------------
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
        _bring_down(interface)
        return False

    # --- Receive -------------------------------------------------------------
    print(f"  Waiting up to {RECV_TIMEOUT} s for echo ...", end="  ", flush=True)
    rx = _poll(bus, time.monotonic() + RECV_TIMEOUT)
    bus.shutdown()
    _bring_down(interface)

    if rx is None:
        print("FAIL")
        print()
        if physical:
            print("  → No frame received.  Check wiring:")
            print("    • CAN_H  ──[120 Ω]── CAN_L  on the SN65HVD230 terminal")
            print("    • 3.3 V power supplied to the SN65HVD230 board")
            print("    • TX/RX pins connected to Jetson CAN controller pads")
        else:
            print("  → Kernel loopback frame not received.")
            print("    Check that the kernel CAN driver loaded:")
            print("      dmesg | grep -i can")
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
    parser = argparse.ArgumentParser(
        description="Waveshare SN65HVD230 / SocketCAN loopback test")
    parser.add_argument(
        "--physical",
        action="store_true",
        help="Physical loopback (requires 120 Ω across CAN_H/CAN_L on the board)",
    )
    parser.add_argument(
        "--interface", default="can0", metavar="IFACE",
        help="SocketCAN interface name (default: can0)",
    )
    args = parser.parse_args()

    ok = run_loopback(interface=args.interface, physical=args.physical)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
