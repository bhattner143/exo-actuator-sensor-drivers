"""07 - Impedance "spring return" for AK60-6 V3.0 KV80.

Anchors a soft virtual spring at q = 0.  Push the shaft away by hand
and feel it spring back.  Demonstrates back-drivable MIT impedance
control.

Run:
  sudo python3 tests/cubemars/07_impedance_spring_return.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v3_bench

KP = 10.0     # spring stiffness (0..500); raise for stiffer hold
KD = 0.5      # damping          (0..5; never 0)
DURATION_S = 30.0
CONTROL_HZ = 100

with open_cubemars_ak_v3_bench(set_zero=True) as bus:
    print(f"Spring-return impedance: anchor=0 rad, Kp={KP}, Kd={KD}")
    print(f"Push the shaft -- it should return to zero.  Ctrl-C to stop.\n")

    dt = 1.0 / CONTROL_HZ
    t_start = time.monotonic()
    last_print = 0.0
    try:
        while time.monotonic() - t_start < DURATION_S:
            t_loop = time.monotonic()
            bus.write("goal_position", {"j1": 0.0}, kp=KP, kd=KD)
            q, dq, ia = bus.read_state(timeout=0.02)["j1"]
            t = t_loop - t_start
            if t - last_print >= 0.25:
                print(f"  t={t:6.2f}s  q={math.degrees(q):+7.2f} deg  "
                      f"dq={dq:+5.2f} rad/s  I={ia:+5.2f} A")
                last_print = t
            sleep_for = dt - (time.monotonic() - t_loop)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        pass

    print("\nDone.  Releasing motor.")
