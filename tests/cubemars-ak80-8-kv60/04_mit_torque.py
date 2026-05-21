"""04_mit_torque.py -- MIT open-loop torque for AK80-8 KV60 V1.x.

Torque mode: kp = 0, kd = 0, t_ff = desired torque (N·m).  Hardware
config is read from ``motor_config.DEFAULT_AK80_8_BENCH_CONFIG``.

SAFETY
------
- AK80-8 T_MAX = ±32 N·m; even small values (0.5 N·m) spin fast unloaded.
- Keep |tau| ≤ 1 N·m for bench testing without a load.
- Brief pulses only.  Zero-torque cooldown between each pulse.
- Have Ctrl-C ready; __exit__ exits MIT mode automatically.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/04_mit_torque.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

PULSE_S    = 0.4
COOL_S     = 0.6
CONTROL_HZ = 100

# AK80-8 T_MAX = ±32 N·m.  Keep values small for unloaded testing.
PULSES_NM = [+0.5, -0.5, +1.0, 0.0]


def main() -> None:
    print(f"AK80-8 KV60 MIT open-loop torque  Kp=0  Kd=0")
    print(f"Pulses (N·m): {PULSES_NM}\n")

    dt = 1.0 / CONTROL_HZ

    with open_cubemars_ak_v1_bench() as bus:
        for tau in PULSES_NM:
            print(f"Torque pulse: {tau:+5.2f} N·m  ({PULSE_S:.1f} s)")

            # -- Pulse ---------------------------------------------------
            t0 = time.monotonic()
            while time.monotonic() - t0 < PULSE_S:
                t_loop = time.monotonic()
                bus.write("goal_torque", {"j1": tau})
                q, dq, ia = bus.read_state()["j1"]
                time.sleep(max(0.0, dt - (time.monotonic() - t_loop)))

            print(f"  end: pos={math.degrees(q):+7.2f} deg  "
                  f"vel={dq:+6.3f} rad/s  "
                  f"I={ia:+5.2f} A")

            # -- Cooldown (zero torque) -----------------------------------
            t0 = time.monotonic()
            while time.monotonic() - t0 < COOL_S:
                t_loop = time.monotonic()
                bus.write("goal_torque", {"j1": 0.0})
                time.sleep(max(0.0, dt - (time.monotonic() - t_loop)))

            print()

    print("Done.  Motor released.")


if __name__ == "__main__":
    main()

