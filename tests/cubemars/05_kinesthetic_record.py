"""05 - Kinesthetic recording for AK60-6 V3.0 KV80.

Soft impedance lets the operator hand-guide the motor while the script
records (t, q, dq, i) at 100 Hz to ``demo_trajectory_ak80.csv``.

Recipe:
  - Use MIT mode with low Kp (~5) and low Kd (~0.4) so the motor offers
    only mild resistance.
  - Set q_des = q_meas each tick so the spring force stays near zero.
  - Press Ctrl-C to stop; CSV is written to repo root.

Run:
  sudo python3 tests/cubemars/05_kinesthetic_record.py
"""
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_ak_v3_bench

KP = 5.0            # soft spring
KD = 0.4            # mild damping  (NEVER 0)
CONTROL_HZ = 100
CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'demo_trajectory_ak80.csv')

with open_ak_v3_bench(set_zero=True) as bus:
    print(f"Recording at {CONTROL_HZ} Hz with Kp={KP} Kd={KD}.")
    print("Hand-guide the motor.  Press Ctrl-C to stop and save.\n")
    rows = []
    dt = 1.0 / CONTROL_HZ
    t_start = time.monotonic()
    try:
        while True:
            t_loop = time.monotonic()
            q, dq, ia = bus.read_state(timeout=0.02)["j1"]
            # q_des = q_meas -> spring stays at the current pose
            bus.write("goal_position", {"j1": q}, kp=KP, kd=KD)
            t = t_loop - t_start
            rows.append((t, q, dq, ia))
            if len(rows) % 50 == 0:
                print(f"  t={t:6.2f}s  q={math.degrees(q):+7.2f} deg  "
                      f"dq={dq:+5.2f} rad/s  I={ia:+5.2f} A")
            # Pace the loop
            sleep_for = dt - (time.monotonic() - t_loop)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        pass

    print(f"\nCaptured {len(rows)} samples.  Writing {CSV_PATH} ...")
    with open(CSV_PATH, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(["t_s", "q_rad", "dq_rad_s", "current_a"])
        w.writerows(rows)
    print("Done.")
