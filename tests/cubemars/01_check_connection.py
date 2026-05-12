"""01 - Verify CubeMars AK80-9 is reachable on the CAN bus.

Sends a null MIT frame (Kp=Kd=tau=0, q_des=0) and reads back the
feedback.  Confirms wiring, CAN ID, and that the motor responds.

Hardware assumed (DEFAULT_CUBEMARS_BENCH_CONFIG):
  Motor  : CubeMars AK80-9 KV60, CAN ID = 0x67 (103 decimal)
  Adapter: HDSC USB-to-CAN on /dev/ttyACM0 @ 921600 bps
  Bus    : CAN 1 Mbps

If you see all zeros, check:
  - Motor powered (18-52 V)
  - CAN H/L polarity not swapped
  - CAN ID matches what was written via R-Link (see motor_config.py)
  - 120 Ω termination resistor present
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

with open_cubemars_bus() as bus:
    print("Polling AK80-9 for 5 feedback packets...\n")
    for i in range(5):
        q, dq, tau = bus.read_state()["j1"]
        print(f"  [{i+1}] q={q:+7.3f} rad  dq={dq:+7.3f} rad/s  tau={tau:+6.3f} N.m")
        time.sleep(0.2)

    q, dq, tau = bus.read_state()["j1"]
    if q == 0.0 and dq == 0.0 and tau == 0.0:
        print("\nWARNING: all zeros -- likely no feedback received.")
        print("  Check: motor power, CAN ID (0x67), wiring, termination.")
    else:
        print("\nMotor responding. CAN connection OK.")
