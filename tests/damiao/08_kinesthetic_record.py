"""08 - Kinesthetic teaching / hand guiding via DamiaoBus.

Sets low Kp/Kd so the motor is compliant.  Records q, dq, tau at 100 Hz
to demo_trajectory.csv using ``bus.read_state()`` (one CAN round-trip
per tick, not three).

Zero-force spring trick: set q_des = q_meas every tick via
``bus.write("mit_command", ...)``.  The spring term Kp*(q_des-q_meas)
stays at zero so only the Kd damping resists motion.
"""
import sys
import os
import time
import csv
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_bus

KP = 20.0
KD = 0.5
RECORD_HZ = 100
SAVE_FILE = "demo_trajectory.csv"

trajectory = []
with open_bus() as bus:
    print(f"[KP={KP}, KD={KD}] Move the motor by hand. Ctrl+C to stop.\n")
    t0 = time.time()
    try:
        while True:
            t_loop = time.time()

            # One CAN round-trip per tick -- get q, dq, tau together
            q, dq, tau = bus.read_state()["j1"]
            t = t_loop - t0

            # Zero-force spring: q_des = q_meas -> spring term cancels.
            bus.write("mit_command", {"j1": q}, kp=KP, kd=KD)
            trajectory.append([round(t, 4), round(q, 5),
                               round(dq, 5), round(tau, 5)])

            elapsed = time.time() - t_loop
            remaining = (1.0 / RECORD_HZ) - elapsed
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print(f"\nStopped. Recorded {len(trajectory)} samples "
              f"({len(trajectory)/RECORD_HZ:.1f} s).")

with open(SAVE_FILE, "w", newline="") as f:
    csv.writer(f).writerows([["t", "q", "dq", "tau"]] + trajectory)
print(f"Saved {SAVE_FILE}")
