"""01 - Verify CubeMars AK60-6 V3.0 KV80 is reachable on the CAN bus.

V3.0 firmware uses CAN 2.0B extended frames -- this script targets the
SocketCAN ``can1`` interface bound to the DSDTech SH-C30A (gs_usb).

Hardware assumed:
  Motor   : CubeMars AK60-6 V3.0 KV80, ESC_ID = 0x02
  Adapter : DSDTech SH-C30A (USB 1d50:606f) on can1 @ 1 Mbps
  Driver  : gs_usb (see install_gs_usb.sh) + SocketCAN

Run:
  sudo python3 tests/cubemars/01_check_connection.py

If can1 is not up yet:
  sudo ip link set can1 up type can bitrate 1000000

Expected: 5 feedback frames with pos/spd/current/temp/err (err=0).
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v3_bench

with open_cubemars_ak_v3_bench() as bus:
    print("Polling AK60-6 V3.0 for 5 feedback packets on can1...\n")
    got = 0
    for i in range(5):
        pos, erpm, ia, tc, err = bus.read_raw(timeout=0.2)["j1"]
        if not (pos == 0.0 and erpm == 0.0 and ia == 0.0 and tc == 0):
            got += 1
        print(f"  [{i+1}] pos={pos:+8.2f} deg  spd={erpm:+8.1f} ERPM  "
              f"I={ia:+6.2f} A  T={tc:3d} C  err={err}")
        time.sleep(0.2)

    if got == 0:
        print("\nWARNING: no feedback received.")
        print("  - Confirm motor is powered (24-48 V) and CAN H/L correct.")
        print("  - Confirm ESC_ID = 0x02 (set via R-Link).")
        print("  - Confirm can1 is up: ip -details link show can1")
        print("  - Bus may be ERROR-PASSIVE; bounce it:")
        print("      sudo ip link set can1 down && "
              "sudo ip link set can1 up type can bitrate 1000000")
    else:
        print(f"\nMotor responding ({got}/5 frames). V3.0 CAN comm OK.")
