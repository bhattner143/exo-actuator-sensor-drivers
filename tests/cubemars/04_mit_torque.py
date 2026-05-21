"""04 - MIT open-loop torque for AK60-6 V3.0 KV80.

Torque mode: Kp = 0, Kd = 0, tau_ff = desired torque (N.m).
Pure open-loop -- with no load the motor will accelerate freely.

SAFETY:
  - Keep |tau| <= 1 N.m unloaded.  AK60-6 T_MAX is ±12 N.m but unloaded
    even 0.3 N.m spins fast.
  - Brief pulses only; never hold tau_ff for long.
  - Have the kill switch (Ctrl-C) ready.

Run:
  sudo python3 tests/cubemars/04_mit_torque.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v3_bench

PULSE_S    = 0.4
COOL_S     = 0.6
PULSES_NM  = [+0.3, -0.3, +0.5, 0.0]   # keep magnitudes small

with open_cubemars_ak_v3_bench() as bus:
    print(f"AK60-6 MIT open-loop torque  Kp=0 Kd=0")
    print(f"Pulses (N.m): {PULSES_NM}\n")

    for tau in PULSES_NM:
        # Pulse
        t0 = time.monotonic()
        while time.monotonic() - t0 < PULSE_S:
            bus.write("goal_torque", {"j1": tau})
            q, dq, ia = bus.read_state(timeout=0.02)["j1"]
            time.sleep(0.01)
        print(f"  tau={tau:+5.2f} Nm  end q={math.degrees(q):+7.2f} deg  "
              f"dq={dq:+6.2f} rad/s  I={ia:+5.2f} A")

        # Cooldown: zero torque so the motor decelerates / coasts
        t0 = time.monotonic()
        while time.monotonic() - t0 < COOL_S:
            bus.write("goal_torque", {"j1": 0.0})
            time.sleep(0.01)

    print("\nDone.  Motor released.")
