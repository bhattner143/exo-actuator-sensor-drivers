"""03 - MIT velocity tracking for AK60-6 V3.0 KV80.

Velocity mode: Kp = 0, Kd > 0, dq_des = target.  V_MAX = ±60 rad/s,
but we keep things tame (±3 rad/s) for safety on the bench.

Run:
  sudo python3 tests/cubemars/03_mit_velocity.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_ak_v3_bench

KD = 1.0          # damping  (0..5) -- acts as the velocity-loop gain in MIT
RUN_S = 2.0       # seconds per setpoint

# Modest, alternating direction.  AK60-6 V3.0 V_MAX = ±60 rad/s.
TARGETS = [+3.0, -3.0, +1.5, 0.0]

with open_ak_v3_bench() as bus:
    print(f"AK60-6 MIT velocity tracking  Kp=0 Kd={KD}")
    print(f"Targets (rad/s): {TARGETS}\n")

    for v_des in TARGETS:
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < RUN_S:
            bus.write("goal_velocity", {"j1": v_des}, kd=KD)
            q, dq, ia = bus.read_state(timeout=0.02)["j1"]
            now = time.monotonic() - t0
            if now - last_print >= 0.2:
                print(f"  dq_des={v_des:+5.2f} rad/s  "
                      f"q={math.degrees(q):+7.2f} deg  "
                      f"dq={dq:+6.2f} rad/s  I={ia:+5.2f} A")
                last_print = now
            time.sleep(0.01)
        print()

    print("Done.  Releasing motor (zero torque).")
