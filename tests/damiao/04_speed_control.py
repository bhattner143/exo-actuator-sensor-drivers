"""04 - Speed mode via DamiaoBus.

    with open_bus(mode=Control_Type.VEL) as bus:
        bus.write("goal_speed", {"j1": v_des})

Delegates to ``DM_CAN.MotorControl.control_Vel`` (CAN ID 0x200 + ESC_ID,
one LE float32).  Output-shaft rad/s.
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_bus, Control_Type

with open_bus(mode=Control_Type.VEL) as bus:
    for vel, label in [(5.0, "+5"), (0.0, "stop"), (-5.0, "-5"), (0.0, "stop")]:
        print(f"v_des = {label} rad/s (output shaft)")
        for _ in range(200):                   # 200 * 10 ms = 2 s
            bus.write("goal_speed", {"j1": vel})
            time.sleep(0.01)
        print(f"   measured vel={bus.read('velocity')['j1']:+.3f}")
