"""02_mit_position.py -- MIT position hold for AK80-8 KV60 V1.x.

Steps through a small set of target angles using MIT position mode
(kp > 0, kd > 0).  Hardware config is read from
``motor_config.DEFAULT_AK80_8_BENCH_CONFIG`` -- no constants here.

SAFETY
------
- Motor must be free to rotate (unloaded), or only holding its own arm.
- Kd MUST be > 0 in position mode -- Kd=0 causes runaway oscillation.
- Targets are modest (±30°) to avoid over-ranging a stiff load.
- AK80-8 V1.x P_MAX = ±12.5 rad.  Targets here are well within range.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/02_mit_position.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

KP         = 4.0    # stiffness  (0..500)
KD         = 1.5    # damping    (0..5)   -- NEVER 0 in position mode
HOLD_S     = 2.0    # seconds to hold each target
CONTROL_HZ = 100

# Small sweep around zero; adjust if mechanical stops are near.
TARGETS_DEG = [0.0, 15.0, -15.0, 30.0, -30.0, 0.0]


def main() -> None:
    print(f"AK80-8 KV60 MIT position hold  Kp={KP}  Kd={KD}")
    print(f"Targets (deg): {TARGETS_DEG}\n")

    dt = 1.0 / CONTROL_HZ

    with open_cubemars_ak_v1_bench(set_zero=True) as bus:
        q, dq, ia = bus.read_state()["j1"]
        print(f"Motor alive  pos={math.degrees(q):+.2f} deg\n")

        for tgt_deg in TARGETS_DEG:
            tgt_rad    = math.radians(tgt_deg)
            t0         = time.monotonic()
            last_print = 0.0
            print(f"Target: {tgt_deg:+6.1f} deg")

            while time.monotonic() - t0 < HOLD_S:
                t_loop = time.monotonic()
                bus.write("goal_position", {"j1": tgt_rad}, kp=KP, kd=KD)
                q, dq, ia = bus.read_state()["j1"]

                now = time.monotonic() - t0
                if now - last_print >= 0.2:
                    err_deg = math.degrees(tgt_rad - q)
                    print(f"  t={now:4.1f}s  "
                          f"tgt={tgt_deg:+6.1f}  "
                          f"pos={math.degrees(q):+7.2f}  "
                          f"err={err_deg:+6.2f} deg  "
                          f"vel={dq:+5.2f} rad/s  "
                          f"I={ia:+5.2f} A")
                    last_print = now

                time.sleep(max(0.0, dt - (time.monotonic() - t_loop)))

            print()

    print("Done.")


if __name__ == "__main__":
    main()

