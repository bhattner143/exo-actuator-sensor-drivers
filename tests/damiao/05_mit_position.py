"""05 - MIT position control via DamiaoBus.

    bus.write("goal_position", {"j1": q_des}, kp=KP, kd=KD)

Maps to ``controlMIT(motor, KP, KD, q_des, 0, 0)``.  Kp > 0, Kd > 0.
NEVER use Kd=0 in position mode (the motor will oscillate).
"""
import sys
import os
import math
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_bus

KP = 30.0   # stiffness  [0, 500]
KD = 1.0    # damping    [0, 5]   -- NEVER 0

# MIT is the default mode; set_zero=True latches q=0 at the current angle.
with open_bus(set_zero=True) as bus:
    for tgt in [math.pi / 2, 0.0, -math.pi / 4, 0.0]:
        print(f"-> {tgt:+.3f} rad (~{math.degrees(tgt):.0f} deg)")
        for _ in range(300):                  # ~3 s @ 100 Hz
            bus.write("goal_position", {"j1": tgt}, kp=KP, kd=KD)
            time.sleep(0.01)
        q, dq, _ = bus.read_state()["j1"]
        print(f"   pos={q:+.3f}  vel={dq:+.3f}")
