"""03 - MIT velocity control on the CubeMars AK80-9.

    bus.write("goal_velocity", {"j1": v_des}, kd=KD)

Maps to: controlMIT(kp=0, kd=KD, q_des=0, dq_des=v_des, tau_ff=0)
Only the damping term acts (Kp is forced to 0).

AK80-9 V_MAX = 50 rad/s (~477 rpm output shaft).
Keep test speeds well below that; 3-6 rad/s is a safe bench test.
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

KD = 1.0     # damping [0, 5]

with open_cubemars_bus() as bus:
    for dq in [3.0, 0.0, -3.0, 0.0]:
        print(f"v_des = {dq:+.2f} rad/s")
        for _ in range(200):                  # 2 s @ 100 Hz
            bus.write("goal_velocity", {"j1": dq}, kd=KD)
            time.sleep(0.01)
        print(f"   measured vel={bus.read('velocity')['j1']:+.3f}")
