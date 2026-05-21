"""00_sniff_motor.py -- Passive sniffer for AK80-8 KV60 (V1.x firmware).

Listens on can1 for CAN 2.0A standard frames with arbitration ID = 0x01
(the motor's ESC_ID).  Does NOT transmit -- completely safe to run with
or without the motor powered on.

If the motor is already in a mode that broadcasts (e.g. Servo mode in
periodic-feedback configuration via R-Link), frames will appear here.
In MIT mode the motor is silent until commanded; use 00_probe_motor.py
for that case.

What to expect
--------------
If the motor is alive and auto-broadcasting, you will see lines like::

  [0.142 s]  ID=0x001  len=8  data: 01 7F FF 7F F7 FF 22 00
              → pos=  -0.001 rad   vel=   0.000 rad/s   I=  -0.003 A
                temp=  -6 °C  err=0x00

If nothing appears within LISTEN_SEC: the motor is silent (normal for
MIT mode), or powered off, or the CAN adapter is not up.

Pre-requisites
--------------
  sudo ip link set can1 up type can bitrate 1000000
  # (the udev rule in install_gs_usb.sh does this automatically)

Run:
  sudo python3 tests/cubemars-ak80-8-kv60/00_sniff_motor.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))

import can
from cubemars.ak_v1.AK_V1_CAN import CubeMarsAkV1Motor

CHANNEL    = "can1"
MOTOR_ID   = 0x01
LISTEN_SEC = 10.0
POLL_CHUNK = 0.05   # recv timeout per iteration


def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def main() -> None:
    print(f"Sniffing {CHANNEL} for std frames  ID=0x{MOTOR_ID:03X}  "
          f"({LISTEN_SEC:.0f} s) …")
    print("Press Ctrl-C to stop early.\n")

    try:
        with can.Bus(channel=CHANNEL, interface="socketcan") as bus:
            motor   = CubeMarsAkV1Motor(bus, can_id=MOTOR_ID)
            t_start = time.monotonic()
            count   = 0

            while time.monotonic() - t_start < LISTEN_SEC:
                msg = bus.recv(timeout=POLL_CHUNK)
                if msg is None:
                    continue

                # Show every standard frame with our motor's ID
                if msg.is_extended_id or msg.arbitration_id != MOTOR_ID:
                    continue

                elapsed = time.monotonic() - t_start
                print(f"[{elapsed:7.3f} s]  ID=0x{msg.arbitration_id:03X}  "
                      f"len={msg.dlc}  data: {_hex(msg.data)}")

                if motor.parse_feedback(msg):
                    print(f"           → pos={motor.pos_rad:+8.3f} rad   "
                          f"vel={motor.vel_rad_s:+8.3f} rad/s   "
                          f"I={motor.current_a:+7.3f} A")
                    print(f"             temp={motor.temp_c:3d} °C  "
                          f"err=0x{motor.error:02X}")
                    count += 1
                else:
                    print(f"           → (not a feedback frame -- data[0] "
                          f"= 0x{msg.data[0]:02X}, expected 0x{MOTOR_ID:02X})")

    except KeyboardInterrupt:
        pass

    print(f"\nFinished.  {count} feedback frame(s) decoded.")


if __name__ == "__main__":
    main()
