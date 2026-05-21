"""01_check_connection.py -- Confirm two-way CAN comms with AK80-8 KV60 V1.x.

Polls N zero-torque feedback frames via the CubeMarsAkV1Bench wrapper and prints
a summary table.  Hardware config is read from
``motor_config.DEFAULT_AK80_8_BENCH_CONFIG`` -- no constants here.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/01_check_connection.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from _common import open_cubemars_ak_v1_bench

N_FRAMES   = 10
POLL_HZ    = 20


def main() -> None:
    print(f"AK80-8 KV60  connection check  ({N_FRAMES} frames)\n")
    print(f"{'Frame':>5}  {'pos (deg)':>10}  {'vel (rad/s)':>11}  "
          f"{'cur (A)':>8}  {'temp (°C)':>9}  {'err':>4}")
    print("-" * 58)

    received = 0
    dt = 1.0 / POLL_HZ

    with open_cubemars_ak_v1_bench() as bus:
        for i in range(N_FRAMES):
            bus.write("mit_command", {"j1": 0.0})   # zero-torque ping
            pos_rad, vel, ia, temp, err = bus.read_raw()["j1"]
            received += 1
            print(f"{i + 1:>5}  {math.degrees(pos_rad):>+10.3f}  "
                  f"{vel:>+11.4f}  "
                  f"{ia:>+8.3f}  "
                  f"{temp:>9d}  "
                  f"{'OK' if err == 0 else hex(err):>4}")
            time.sleep(max(0.0, dt - 0.005))

    print("-" * 58)
    print(f"Received {received}/{N_FRAMES} frames "
          f"({'OK' if received == N_FRAMES else 'DEGRADED -- check CAN link'}).")


if __name__ == "__main__":
    main()

