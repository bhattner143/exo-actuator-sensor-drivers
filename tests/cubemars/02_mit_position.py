"""02 - MIT position hold for AK60-6 V3.0 KV80.

Steps through several target angles using MIT mode (Kp/Kd position).
V3.0 firmware MIT byte order is Kp-first (handled by AkV3Motor.set_mit).

SAFETY:
  - Motor must be free to rotate (unloaded), or holding only its own arm.
  - Kd MUST be > 0 in position mode -- Kd=0 will cause runaway oscillation.
  - Position is in OUTPUT-SHAFT radians.  Limits: P_MAX = ±12.56 rad.

Run:
  sudo python3 tests/cubemars/02_mit_position.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_ak_v3_bench

KP = 60.0        # stiffness  (0..500) -- AK60-6 V3.0 tracks slowly below ~40
KD = 1.5         # damping    (0..5)  -- NEVER set to 0 here
HOLD_S = 1.5     # seconds to hold each target
CONTROL_HZ = 100

# Modest sweep around the current resting position; do not chase ±2pi at once.
TARGETS_DEG = [0.0, 15.0, -15.0, 30.0, 0.0]

with open_ak_v3_bench(set_zero=True) as bus:
    print(f"AK60-6 MIT position hold  Kp={KP} Kd={KD}")
    print(f"Targets (deg): {TARGETS_DEG}\n")

    dt = 1.0 / CONTROL_HZ
    for tgt_deg in TARGETS_DEG:
        tgt_rad = math.radians(tgt_deg)
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < HOLD_S:
            bus.write("goal_position", {"j1": tgt_rad}, kp=KP, kd=KD)
            q, dq, ia = bus.read_state(timeout=0.02)["j1"]
            now = time.monotonic() - t0
            if now - last_print >= 0.2:
                print(f"  tgt={tgt_deg:+6.1f} deg  "
                      f"q={math.degrees(q):+7.2f} deg  "
                      f"dq={dq:+6.2f} rad/s  I={ia:+5.2f} A")
                last_print = now
            time.sleep(max(0.0, dt - 0.005))
        print()

    print("Done.  Releasing motor (zero torque).")
