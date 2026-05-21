"""00_probe_motor.py -- Minimal V1.x probe: enter MIT mode, send one zero-torque frame.

This is the safest first test after the passive sniffer.  It sends exactly
two CAN frames:
  1. Enter MIT mode  (0xFF×7 + 0xFC)
  2. One MIT command with kp=kd=0, p_des=0, v_des=0, t_ff=0  (zero torque)

A zero-torque MIT frame generates no motion; it only requests a feedback
reply, confirming two-way communication.

If the motor responds you will see its current position, speed, current,
temperature, and error flag.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/00_probe_motor.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))

import can
from cubemars.ak_v1.AK_V1_CAN import CubeMarsAkV1Motor

CHANNEL  = "can1"
MOTOR_ID = 0x01


def main() -> None:
    print(f"Probing AK80-8 KV60 on {CHANNEL}  ESC_ID=0x{MOTOR_ID:02X}")

    try:
        bus   = can.Bus(channel=CHANNEL, interface="socketcan")
        motor = CubeMarsAkV1Motor(bus, can_id=MOTOR_ID)

        # ── 1. Enter MIT mode ───────────────────────────────────────────────
        print("  Sending Enter MIT mode (0xFF×7 + 0xFC) …")
        motor.enter_mode()
        time.sleep(0.02)   # brief settle

        # ── 2. Zero-torque MIT frame (no motion) ────────────────────────────
        print("  Sending zero-torque MIT frame (kp=0 kd=0 t_ff=0) …")
        motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)

        # ── 3. Wait for feedback ────────────────────────────────────────────
        got = motor.poll_feedback(timeout=0.5)

        if got:
            print("\n  ✓ Motor replied!")
            print(f"    pos   = {motor.pos_rad:+.4f} rad")
            print(f"    vel   = {motor.vel_rad_s:+.4f} rad/s")
            print(f"    cur   = {motor.current_a:+.4f} A")
            print(f"    temp  = {motor.temp_c} °C")
            print(f"    error = 0x{motor.error:02X}"
                  + (" (OK)" if motor.error == 0 else " *** ERROR ***"))
        else:
            print("\n  ✗ No feedback within 0.5 s.")
            print("    Check: motor powered? CAN ID correct? can1 up at 1 Mbps?")

    finally:
        try:
            motor.exit_mode()
            time.sleep(0.01)
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass

    print("\nDone.")


if __name__ == "__main__":
    main()
