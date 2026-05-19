"""Probe whether the HDSC adapter can send extended-ID frames to a V3.0 motor.

Tries both candidate values of byte[8] (the DLC/flags byte) of the HDSC
30-byte envelope -- 0x12 and 0x1a -- and prints every raw HDSC reply
received, with the decoded 4-byte CAN ID.

If the V3.0 motor replies, you'll see frames whose CAN ID is
``(0x29 << 8) | ESC_ID`` = ``0x2902`` for ESC=0x02.

If the adapter firmware doesn't support extended-frame TX at all, you'll
see no feedback (or only echo frames with cmd != 0x11).

Run:
    python tests/cubemars/00_probe_v3_serial.py
"""
from __future__ import annotations

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from ak_v3_serial import (
    AkV3Serial, _BYTE8_EXT_PRIMARY, _BYTE8_EXT_FALLBACK,
)

CAN_ID = 0x02            # V3.0 motor ESC_ID (from R-Link)
PORT   = "/dev/ttyACM0"


def probe(flag_byte: int) -> None:
    label = f"byte[8]=0x{flag_byte:02X}"
    print(f"\n========== Probing with {label} ==========")
    with AkV3Serial(port=PORT, can_id=CAN_ID,
                    model="AK60-6", ext_flag_byte=flag_byte) as m:
        # Flush stale data
        m._read_frames()

        # Send a zero-everything MIT frame -- the safest possible ping.
        # (In V3.0, this commands p=0, v=0, kp=0, kd=0, tau=0 -> no motion.)
        print("--> sending MIT zero ping (extended ID 0x0802)")
        m.set_mit()
        time.sleep(0.1)

        frames = m.dump_raw(duration=0.5)
        print(f"<-- got {len(frames)} HDSC reply frames")
        for i, f in enumerate(frames):
            cmd    = f[1]
            can_id = (f[6] << 24) | (f[5] << 16) | (f[4] << 8) | f[3]
            print(f"   [{i:2d}] cmd=0x{cmd:02X}  can_id=0x{can_id:08X}  raw={f.hex()}")

        # Try parsing
        ok = any(m._parse_one(f) for f in frames)
        if ok:
            print(f"  *** V3.0 FEEDBACK DECODED ***  pos={m.pos_deg:+.2f} deg, "
                  f"spd={m.spd_erpm:+.1f} ERPM, I={m.current_a:+.2f} A, "
                  f"T={m.temp_c} C, err={m.error}")
        else:
            print("  (no frame matched expected feedback ID 0x2902)")


def main() -> None:
    print(f"Probing CubeMars V3.0 motor via HDSC at {PORT}, ESC_ID=0x{CAN_ID:02X}")
    print("Make sure the motor is powered (18-52 V) and the CAN H/L wires")
    print("are connected to the HDSC adapter, with 120 ohm termination.")
    print()

    # Try both candidate extended-frame flag bytes.
    for flag in (_BYTE8_EXT_PRIMARY, _BYTE8_EXT_FALLBACK):
        try:
            probe(flag)
        except Exception as e:
            print(f"  ERROR with byte[8]=0x{flag:02X}: {e}")

    print("\nDone.  If neither probe got cmd=0x11 frames with can_id=0x2902,")
    print("the HDSC adapter firmware probably does not transmit 29-bit frames,")
    print("and you'll need the Canable / Waveshare adapter for V3.0.")


if __name__ == "__main__":
    main()
