"""10 - Impedance spring-return via DamiaoBus.

Motor behaves like a torsional spring anchored at ORIGIN.  Push the
shaft away, it springs back.

Control law (firmware MIT):
    tau_cmd = KP * (ORIGIN - q) + KD * (0 - dq)

KP (N.m/rad)  -- spring stiffness, [0, 500]
  *  10-20  : soft / back-drivable
  *  30-60  : moderate restoring force
  * 100+    : stiff
KD (N.m.s/rad) -- damping, [0, 5]
  * ~KP/30 : light damping (springy return)
  * ~KP/15 : critically damped (no overshoot)
  NEVER use KD = 0 in position mode.
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_bus

KP          = 5.0
KD          = 0.5
ORIGIN      = 0.0
LOOP_HZ     = 100
RUN_SECONDS = 30

LOOP_DT = 1.0 / LOOP_HZ
N_STEPS = int(RUN_SECONDS * LOOP_HZ)

with open_bus(set_zero=True) as bus:
    print(f"Spring-return active for {RUN_SECONDS} s  (KP={KP}, KD={KD})")
    print("Push the shaft away and watch it return.  Ctrl+C to stop early.\n")

    for step in range(N_STEPS):
        t_start = time.time()

        # One CAN round-trip per tick (was three before).
        q, dq, tau = bus.read_state()["j1"]

        # Spring-return: q_des = ORIGIN every tick.
        bus.write("goal_position", {"j1": ORIGIN}, kp=KP, kd=KD)

        if step % (LOOP_HZ // 2) == 0:
            print(f"  t={step * LOOP_DT:6.1f}s  "
                  f"q={q:+6.3f} rad  dq={dq:+6.3f} rad/s  tau={tau:+5.2f} N.m")

        elapsed = time.time() - t_start
        sleep_t = LOOP_DT - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    print("\nSpring-return demo complete.")
