"""05_kinesthetic_record.py -- Hand-guide and record for AK80-8 KV60 V1.x.

Soft MIT impedance (low Kp, low Kd) lets the operator hand-guide the
shoulder motor while the script records (t, q, dq, current) at 100 Hz
to ``demo_trajectory_ak80_8.csv``.  Hardware config is read from
``motor_config.DEFAULT_AK80_8_BENCH_CONFIG``.

Recipe
------
- Kp = 5 and Kd = 0.4 give mild resistance; motor follows your hand.
- q_des = q_meas each tick → spring force ≈ 0, motor feels compliant.
- Press Ctrl-C to stop; CSV is written to the repo root.
- Replay the recording with ``06_replay_trajectory.py``.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/05_kinesthetic_record.py
"""
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

KP         = 5.0    # soft spring -- motor feels back-drivable
KD         = 0.4    # mild damping; NEVER 0
CONTROL_HZ = 100

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'demo_trajectory_ak80_8.csv')


def main() -> None:
    print(f"Recording at {CONTROL_HZ} Hz  Kp={KP}  Kd={KD}")
    print(f"CSV → {os.path.normpath(CSV_PATH)}")
    print("Hand-guide the motor.  Press Ctrl-C to stop and save.\n")

    rows = []
    dt   = 1.0 / CONTROL_HZ

    with open_cubemars_ak_v1_bench(set_zero=True) as bus:
        q, dq, ia = bus.read_state()["j1"]
        print(f"Motor alive  pos={math.degrees(q):+.2f} deg\n")

        t_start = time.monotonic()
        try:
            while True:
                t_loop = time.monotonic()
                # q_des = q_meas each tick → spring force ≈ zero
                bus.write("goal_position", {"j1": q}, kp=KP, kd=KD)
                q, dq, ia = bus.read_state()["j1"]

                t = t_loop - t_start
                rows.append((t, q, dq, ia))

                if len(rows) % 50 == 0:
                    print(f"  t={t:6.2f}s  "
                          f"q={math.degrees(q):+7.2f} deg  "
                          f"dq={dq:+5.2f} rad/s  "
                          f"I={ia:+5.2f} A")

                sleep_for = dt - (time.monotonic() - t_loop)
                if sleep_for > 0:
                    time.sleep(sleep_for)

        except KeyboardInterrupt:
            pass

    print(f"\nCaptured {len(rows)} samples.  Writing CSV …")
    with open(CSV_PATH, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(["t_s", "q_rad", "dq_rad_s", "current_a"])
        w.writerows(rows)
    print(f"Saved → {os.path.normpath(CSV_PATH)}")


if __name__ == "__main__":
    main()

