"""06 - Replay a recorded trajectory on the AK60-6 V3.0 KV80.

Reads ``demo_trajectory_ak80.csv`` produced by ``05_kinesthetic_record.py``
and re-issues each (t, q) sample with a stiffer impedance so the motor
follows the recorded path.

Run:
  sudo python3 tests/cubemars/06_replay_trajectory.py
"""
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v3_bench

KP = 30.0           # stiffer for replay (0..500)
KD = 1.0            # damping            (0..5, never 0)
CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'demo_trajectory_ak80.csv')

if not os.path.exists(CSV_PATH):
    print(f"ERROR: {CSV_PATH} not found.  Run 05_kinesthetic_record.py first.")
    sys.exit(1)

samples = []
with open(CSV_PATH, 'r') as f:
    r = csv.DictReader(f)
    for row in r:
        samples.append((float(row["t_s"]),
                        float(row["q_rad"]),
                        float(row["dq_rad_s"])))

if not samples:
    print("ERROR: CSV is empty.")
    sys.exit(1)

print(f"Loaded {len(samples)} samples spanning {samples[-1][0]:.2f} s.")
print(f"Replaying with Kp={KP} Kd={KD}.\n")

with open_cubemars_ak_v3_bench() as bus:
    # Drive to the first sample slowly before starting playback
    t0_q, q0, _ = samples[0]
    print(f"Moving to start q={math.degrees(q0):+7.2f} deg ...")
    t_settle = time.monotonic()
    while time.monotonic() - t_settle < 1.0:
        bus.write("goal_position", {"j1": q0}, kp=KP, kd=KD)
        bus.read_state(timeout=0.02)
        time.sleep(0.01)

    print("Playback start.")
    t_start = time.monotonic()
    last_print = 0.0
    for (t_s, q_des, dq_des) in samples:
        # Wait until wall time catches up to this sample
        while time.monotonic() - t_start < t_s:
            time.sleep(0.001)
        bus.write("goal_position", {"j1": q_des}, kp=KP, kd=KD, dq_des=dq_des)
        q, dq, ia = bus.read_state(timeout=0.005)["j1"]
        if t_s - last_print >= 0.5:
            print(f"  t={t_s:6.2f}s  q_des={math.degrees(q_des):+7.2f}  "
                  f"q={math.degrees(q):+7.2f} deg  I={ia:+5.2f} A")
            last_print = t_s

    print("\nPlayback complete.")
