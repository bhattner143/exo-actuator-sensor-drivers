"""06_replay_trajectory.py -- Replay a recorded trajectory on AK80-8 KV60 V1.x.

Reads ``demo_trajectory_ak80_8.csv`` (produced by ``05_kinesthetic_record.py``)
and re-issues each (t, q) sample with a stiffer impedance.  Hardware config
is read from ``motor_config.DEFAULT_AK80_8_BENCH_CONFIG``.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000
  # Run 05_kinesthetic_record.py first to generate the CSV.

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/06_replay_trajectory.py
"""
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

KP = 30.0    # stiffer for replay (0..500)
KD =  1.0    # damping            (0..5; never 0)

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'demo_trajectory_ak80_8.csv')


def load_csv(path: str) -> list:
    if not os.path.exists(path):
        print(f"ERROR: {path} not found.  Run 05_kinesthetic_record.py first.")
        sys.exit(1)
    samples = []
    with open(path, 'r') as f:
        for row in csv.DictReader(f):
            samples.append((float(row["t_s"]),
                            float(row["q_rad"]),
                            float(row["dq_rad_s"])))
    if not samples:
        print("ERROR: CSV is empty.")
        sys.exit(1)
    return samples


def main() -> None:
    samples = load_csv(CSV_PATH)
    print(f"Loaded {len(samples)} samples spanning {samples[-1][0]:.2f} s.")
    print(f"Replaying with Kp={KP}  Kd={KD}.\n")

    with open_cubemars_ak_v1_bench() as bus:
        # Drive to the first sample position before starting playback
        q0 = samples[0][1]
        print(f"Moving to start  q0={math.degrees(q0):+.2f} deg …")
        t_settle = time.monotonic()
        while time.monotonic() - t_settle < 1.5:
            bus.write("goal_position", {"j1": q0}, kp=KP, kd=KD)
            time.sleep(0.01)

        print("Playback start.")
        t_start    = time.monotonic()
        last_print = 0.0

        for (t_s, q_des, dq_des) in samples:
            while time.monotonic() - t_start < t_s:
                time.sleep(0.001)
            bus.write("goal_position", {"j1": q_des}, kp=KP, kd=KD,
                      dq_des=dq_des)
            q, dq, ia = bus.read_state()["j1"]

            if t_s - last_print >= 0.5:
                print(f"  t={t_s:6.2f}s  "
                      f"q_des={math.degrees(q_des):+7.2f}  "
                      f"q={math.degrees(q):+7.2f} deg  "
                      f"I={ia:+5.2f} A")
                last_print = t_s

    print("\nPlayback complete.")


if __name__ == "__main__":
    main()

