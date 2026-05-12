"""01 - Verify CubeMars AK60-6 V3.0 KV80 is reachable on the CAN bus.

Sends a null MIT frame (Kp=Kd=tau=0, q_des=0) and reads back the
feedback.  Confirms wiring, CAN ID, and that the motor responds.

Hardware assumed (DEFAULT_CUBEMARS_BENCH_CONFIG):
  Motor  : CubeMars AK60-6 V3.0 KV80, CAN ID = 0x68 (104 decimal)
  Adapter: HDSC USB-to-CAN on /dev/ttyACM0 @ 921600 bps
  Bus    : CAN 1 Mbps

>>> R-Link Application Configuration required <<<
  CAN Mode  : Inquiry Feedback  (= MIT mode)
              NOT "Periodic Feedback" -- that is Servo mode and will
              NOT respond to this driver.
  CAN Bitrate: 1 Mbps
  CAN ID    : 104
  Click "Write" in R-Link, then power-cycle the motor.

If you see all zeros, check:
  - CAN Mode = Inquiry Feedback in R-Link (most common cause)
  - Motor powered (18-52 V)
  - CAN H/L polarity not swapped
  - CAN ID = 104 (0x68) in motor_config.py
  - 120 Ohm termination resistor present
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

with open_cubemars_bus() as bus:
    print("Polling AK60-6 for 5 feedback packets...\n")
    for i in range(5):
        q, dq, tau = bus.read_state()["j1"]
        print(f"  [{i+1}] q={q:+7.3f} rad  dq={dq:+7.3f} rad/s  tau={tau:+6.3f} N.m")
        time.sleep(0.2)

    q, dq, tau = bus.read_state()["j1"]
    if q == 0.0 and dq == 0.0 and tau == 0.0:
        print("\nWARNING: all zeros -- likely no feedback received.")
        print("  Most common cause: CAN Mode = 'Periodic Feedback' (Servo mode).")
        print("  Set CAN Mode = Inquiry Feedback in R-Link, click Write, power-cycle.")
        print("  Other checks: motor power, CAN ID (0x68=104), wiring, 120 Ohm term.")
    else:
        print("\nMotor responding. CAN connection OK.")
