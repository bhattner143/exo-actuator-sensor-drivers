"""04 - MIT open-loop torque on the CubeMars AK80-9.

    bus.write("goal_torque", {"j1": tau})

Maps to: controlMIT(kp=0, kd=0, q_des=0, dq_des=0, tau_ff=tau)
No feedback -- the shaft accelerates freely under the commanded torque.

WARNING: AK80-9 T_MAX = 18 N.m.  Keep |tau| <= 1 N.m for bench tests
without a load; the motor will spin up quickly at higher values.
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

TAU_NM   = 0.5     # N.m on output shaft (keep <= 1 N.m unloaded)
DURATION = 1.5     # seconds per step

with open_cubemars_bus() as bus:
    for tau in [TAU_NM, -TAU_NM]:
        print(f"t_ff = {tau:+.2f} N.m for {DURATION:.1f} s")
        t0 = time.time()
        while time.time() - t0 < DURATION:
            bus.write("goal_torque", {"j1": tau})
            time.sleep(0.01)
        q, dq, _ = bus.read_state()["j1"]
        print(f"   end vel={dq:+.3f}  pos={q:+.3f}")
