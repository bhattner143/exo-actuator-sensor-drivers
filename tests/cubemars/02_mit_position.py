"""02 - MIT position control on the CubeMars AK60-6 V3.0 KV80.

    bus.write("goal_position", {"j1": q_des}, kp=KP, kd=KD)

Maps to the MIT bit-packed CAN frame:
    controlMIT(kp=KP, kd=KD, q_des=q_des, dq_des=0, tau_ff=0)

NEVER use Kd=0 in position mode -- the motor will oscillate or runaway.
Start with low Kp on a new motor; AK60-6 takes 30-80 well.

AK60-6 limits: P_MAX=12.5 rad, V_MAX=45 rad/s, T_MAX=15 N.m
"""
import sys
import os
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

KP = 30.0   # stiffness [0, 500]
KD = 1.0    # damping   [0, 5]  -- NEVER 0

# set_zero=True latches q=0 at the current shaft angle on entry.
with open_cubemars_bus(set_zero=True) as bus:
    for tgt in [math.pi / 2, 0.0, -math.pi / 4, 0.0]:
        print(f"-> {tgt:+.3f} rad (~{math.degrees(tgt):.0f} deg)")
        for _ in range(300):                  # ~3 s @ 100 Hz
            bus.write("goal_position", {"j1": tgt}, kp=KP, kd=KD)
            time.sleep(0.01)
        q, dq, _ = bus.read_state()["j1"]
        print(f"   pos={q:+.3f}  vel={dq:+.3f}")
