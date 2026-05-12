"""06 - Replay a recorded trajectory on the CubeMars AK80-9.

Loads demo_trajectory_ak80.csv (written by 05_kinesthetic_record.py) and
tracks it with stiffer gains using goal_position.  Holds the final
position for 2 s at the end.
"""
import sys
import os
import csv
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

KP           = 30.0
KD           = 1.0
LOAD_FILE    = "demo_trajectory_ak80.csv"
SPEED_FACTOR = 1.0    # < 1.0 = slower replay, > 1.0 = faster

with open(LOAD_FILE) as f:
    rows = list(csv.DictReader(f))

if len(rows) < 2:
    print("Not enough data in trajectory file.")
    sys.exit(0)

print(f"Loaded {len(rows)} samples from {LOAD_FILE}")
with open_cubemars_bus() as bus:
    t_prev = float(rows[0]["t"])
    for row in rows:
        t_now = float(row["t"])
        q = float(row["q"])
        dt = (t_now - t_prev) / SPEED_FACTOR
        t_prev = t_now

        bus.write("goal_position", {"j1": q}, kp=KP, kd=KD)
        if dt > 0:
            time.sleep(dt)

    print("Replay complete.")

    # Hold final position for 2 s
    q_final = float(rows[-1]["q"])
    for _ in range(200):
        bus.write("goal_position", {"j1": q_final}, kp=KP, kd=KD)
        time.sleep(0.01)
