"""07_impedance_spring_return.py -- Soft spring anchored at zero for AK80-8 KV60 V1.x.

Anchors a virtual spring at q = 0 rad.  Push the shaft away by hand
and feel it return to zero.  Hardware config is read from
``motor_config.DEFAULT_AK80_8_BENCH_CONFIG``.

Tune KP and KD to taste:
  - Low  Kp (5–15)  → soft, compliant feel; good for exploration.
  - High Kp (40–80) → stiff; snaps back quickly.
  - Kd must always be > 0 to prevent oscillation.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/07_impedance_spring_return.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

KP         = 10.0    # spring stiffness (0..500)
KD         =  0.5    # damping          (0..5;  NEVER 0)
DURATION_S = 30.0
CONTROL_HZ = 100


def main() -> None:
    print(f"AK80-8 KV60 spring-return impedance")
    print(f"  anchor=0 rad  Kp={KP}  Kd={KD}  duration={DURATION_S:.0f} s")
    print("Push the shaft -- it should return to zero.  Ctrl-C to stop.\n")

    dt = 1.0 / CONTROL_HZ

    with open_cubemars_ak_v1_bench(set_zero=True) as bus:
        q, dq, ia = bus.read_state()["j1"]
        print(f"Motor alive  pos={math.degrees(q):+.2f} deg\n")

        t_start    = time.monotonic()
        last_print = 0.0

        try:
            while time.monotonic() - t_start < DURATION_S:
                t_loop = time.monotonic()
                bus.write("goal_position", {"j1": 0.0}, kp=KP, kd=KD)
                q, dq, ia = bus.read_state()["j1"]

                t = t_loop - t_start
                if t - last_print >= 0.25:
                    print(f"  t={t:6.2f}s  "
                          f"q={math.degrees(q):+7.2f} deg  "
                          f"dq={dq:+5.2f} rad/s  "
                          f"I={ia:+5.2f} A")
                    last_print = t

                sleep_for = dt - (time.monotonic() - t_loop)
                if sleep_for > 0:
                    time.sleep(sleep_for)

        except KeyboardInterrupt:
            pass

    print("\nDone.  Motor released.")


if __name__ == "__main__":
    main()

