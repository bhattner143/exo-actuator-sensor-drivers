"""05 - Kinesthetic teaching / hand guiding on the CubeMars AK80-9.

Sets low Kp/Kd so the motor is back-drivable by hand.  Records
(t, q, dq, tau) at 100 Hz to demo_trajectory_ak80.csv.

Zero-force spring trick: set q_des = q_meas every tick.
The spring term Kp*(q_des - q_meas) stays zero; only the Kd damping
resists motion, giving a compliant feel.

Run 06_replay_trajectory.py to replay the recorded motion.
"""
import sys
import os
import time
import csv

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

KP         = 20.0   # low stiffness for back-driveability
KD         = 0.5    # light damping
RECORD_HZ  = 100
SAVE_FILE  = "demo_trajectory_ak80.csv"

trajectory = []
with open_cubemars_bus() as bus:
    print(f"[KP={KP}, KD={KD}] Move the motor by hand. Ctrl+C to stop.\n")
    t0 = time.time()
    try:
        while True:
            t_loop = time.time()

            # One CAN round-trip per tick
            q, dq, tau = bus.read_state()["j1"]
            t = t_loop - t0

            # Zero-force spring: q_des = q_meas -> spring term cancels
            bus.write("mit_command", {"j1": q}, kp=KP, kd=KD)
            trajectory.append([round(t, 4), round(q, 5),
                               round(dq, 5), round(tau, 5)])

            remaining = (1.0 / RECORD_HZ) - (time.time() - t_loop)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print(f"\nStopped. Recorded {len(trajectory)} samples "
              f"({len(trajectory) / RECORD_HZ:.1f} s).")

with open(SAVE_FILE, "w", newline="") as f:
    csv.writer(f).writerows([["t", "q", "dq", "tau"]] + trajectory)
print(f"Saved {SAVE_FILE}")
