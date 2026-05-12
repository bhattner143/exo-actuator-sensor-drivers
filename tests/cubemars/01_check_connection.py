"""01 - Verify CubeMars AK80-9 is reachable on the CAN bus.

Sends a null MIT frame (Kp=Kd=tau=0, q_des=0) and reads back the
feedback.  Confirms wiring, CAN ID, and that the motor responds.

Hardware assumed (DEFAULT_CUBEMARS_BENCH_CONFIG):
  Motor  : CubeMars AK80-9 KV60, CAN ID = 0x04 (4 decimal)
  Adapter: HDSC USB-to-CAN on /dev/ttyACM0 @ 921600 bps
  Bus    : CAN 1 Mbps

>>> R-Link Application Configuration required <<<
  CAN Mode  : MIT          (NOT "Periodic Feedback" -- that is Servo mode
                            and will NOT respond to this driver)
  CAN Bitrate: 1 Mbps
  CAN ID    : 4
  Click "Write" in R-Link, then power-cycle the motor.

If you see all zeros, check:
  - CAN Mode = MIT in R-Link (most common cause of zero feedback)
  - Motor powered (18-52 V)
  - CAN H/L polarity not swapped
  - CAN ID matches what was written via R-Link (see motor_config.py)
  - 120 Ohm termination resistor present
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
        print("  Most common cause: CAN Mode is set to 'Periodic Feedback'")
        print("  (Servo mode) in R-Link. This driver requires 'MIT' mode.")
        print("  Other checks: motor power, CAN ID (0x04), wiring, 120 Ohm term.")
    else:
        print("\nMotor responding. CAN connection OK.")
