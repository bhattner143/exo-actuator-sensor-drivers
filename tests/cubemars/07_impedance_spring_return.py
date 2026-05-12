"""07 - Impedance spring-return on the CubeMars AK80-9.

The motor behaves like a torsional spring anchored at ORIGIN.
Push the shaft away by hand -- it springs back.

Control law (MIT mode):
    tau_cmd = KP * (ORIGIN - q) + KD * (0 - dq)

KP tuning guide (N.m/rad):
    5-15   : soft / back-drivable
    30-60  : moderate restoring force
    100+   : stiff
KD tuning:
    KD ~ KP / 30 : light, springy return
    KD ~ KP / 15 : critically damped, no overshoot
    NEVER KD = 0 (motor oscillates in position mode).

AK80-9 limits: Kp [0, 500], Kd [0, 5], T_MAX 18 N.m
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_bus

KP          = 5.0
KD          = 0.5
ORIGIN      = 0.0
LOOP_HZ     = 100
RUN_SECONDS = 30

LOOP_DT = 1.0 / LOOP_HZ
N_STEPS = int(RUN_SECONDS * LOOP_HZ)

# set_zero=True latches the current angle as ORIGIN on entry.
with open_cubemars_bus(set_zero=True) as bus:
    print(f"Spring-return active for {RUN_SECONDS} s  (KP={KP}, KD={KD})")
    print("Push the shaft away and watch it return.  Ctrl+C to stop early.\n")

    for step in range(N_STEPS):
        t_start = time.time()

        q, dq, tau = bus.read_state()["j1"]
        bus.write("goal_position", {"j1": ORIGIN}, kp=KP, kd=KD)

        if step % (LOOP_HZ // 2) == 0:
            print(f"  t={step * LOOP_DT:6.1f}s  "
                  f"q={q:+6.3f} rad  dq={dq:+6.3f} rad/s  tau={tau:+5.2f} N.m")

        sleep_t = LOOP_DT - (time.time() - t_start)
        if sleep_t > 0:
            time.sleep(sleep_t)

    print("\nSpring-return demo complete.")
