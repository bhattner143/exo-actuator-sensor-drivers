"""06 - MIT velocity control via DamiaoBus.

    bus.write("goal_velocity", {"j1": v_des}, kd=KD)

Maps to ``controlMIT(motor, 0, KD, 0, v_des, 0)``.  Only the damping
term acts.  Different from Speed mode (0x200+ID): MIT velocity uses the
bit-packed frame and lets you tune Kd per call.
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_bus

KD = 1.0     # damping [0, 5]; Kp is forced to 0 by goal_velocity

with open_bus() as bus:
    for dq in [3.0, 0.0, -3.0, 0.0]:
        print(f"v_des = {dq:+.2f} rad/s")
        for _ in range(200):                   # 200 * 10 ms = 2 s
            bus.write("goal_velocity", {"j1": dq}, kd=KD)
            time.sleep(0.01)
        print(f"   measured vel={bus.read('velocity')['j1']:+.3f}")
