"""03 - Position-Speed mode via DamiaoBus.

LeRobot-idiomatic usage:
    with open_bus(mode=Control_Type.POS_VEL, set_zero=True) as bus:
        bus.write("goal_pos_vel", {"j1": p_des}, dq_des=v_des)

Delegates to ``DM_CAN.MotorControl.control_Pos_Vel`` (two LE float32 at
CAN ID 0x100 + ESC_ID).  Output-shaft units.
"""
import sys
import os
import math
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_bus, Control_Type

TARGET_OUT_RAD = math.pi / 2     # +90 degrees on the output shaft
CRUISE_SPEED   = 5.0             # rad/s on the output shaft

with open_bus(mode=Control_Type.POS_VEL, set_zero=True) as bus:
    print(f"-> {TARGET_OUT_RAD:+.3f} rad (~{math.degrees(TARGET_OUT_RAD):.0f} deg)")
    for _ in range(400):                       # ~4 s @ 100 Hz
        bus.write("goal_pos_vel", {"j1": TARGET_OUT_RAD}, dq_des=CRUISE_SPEED)
        time.sleep(0.01)
    print(f"   pos={bus.read('position')['j1']:+.3f}")

    print("-> Back to 0")
    for _ in range(400):
        bus.write("goal_pos_vel", {"j1": 0.0}, dq_des=CRUISE_SPEED)
        time.sleep(0.01)
    print(f"   pos={bus.read('position')['j1']:+.3f}")
