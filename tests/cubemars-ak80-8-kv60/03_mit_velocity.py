"""03_mit_velocity.py -- MIT velocity tracking for AK80-8 KV60 V1.x.

Velocity mode: kp = 0, kd > 0, v_des = target.  Hardware config is read
from ``motor_config.DEFAULT_AK80_8_BENCH_CONFIG``.
V_MAX = ±37.5 rad/s; targets here are kept tame (±3 rad/s) for safety.

SAFETY
------
- Motor must be free to spin (no mechanical load).
- AK80-8 V1.x V_MAX = ±37.5 rad/s.  Keep |v_des| << V_MAX on the bench.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/03_mit_velocity.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

KD         = 1.0     # velocity-loop damping gain (0..5)
RUN_S      = 2.0     # seconds per setpoint
CONTROL_HZ = 100

# Modest, alternating direction.  AK80-8 V1.x V_MAX = ±37.5 rad/s.
TARGETS = [+3.0, -3.0, +1.5, 0.0]


def main() -> None:
    print(f"AK80-8 KV60 MIT velocity tracking  Kp=0  Kd={KD}")
    print(f"Targets (rad/s): {TARGETS}\n")

    dt = 1.0 / CONTROL_HZ

    with open_cubemars_ak_v1_bench() as bus:
        for v_des in TARGETS:
            print(f"Velocity setpoint: {v_des:+5.2f} rad/s")
            t0         = time.monotonic()
            last_print = 0.0

            while time.monotonic() - t0 < RUN_S:
                t_loop = time.monotonic()
                bus.write("goal_velocity", {"j1": v_des}, kd=KD)
                q, dq, ia = bus.read_state()["j1"]

                now = time.monotonic() - t0
                if now - last_print >= 0.2:
                    print(f"  t={now:4.1f}s  "
                          f"v_des={v_des:+5.2f}  "
                          f"pos={math.degrees(q):+7.2f} deg  "
                          f"vel={dq:+6.3f} rad/s  "
                          f"I={ia:+5.2f} A")
                    last_print = now

                time.sleep(max(0.0, dt - (time.monotonic() - t_loop)))

            print()

    print("Done.  Motor released.")


if __name__ == "__main__":
    main()

