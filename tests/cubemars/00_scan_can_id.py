"""00 - Scan a range of CAN IDs to find which one the CubeMars motor uses.

Sends a null MIT poll frame (Kp=Kd=tau=0, q_des=0) to each candidate ID and
listens for any feedback frame on the HDSC USB-to-CAN adapter.  Reports
every CAN ID that produces a response.

When to use:
  - You changed the CAN ID in R-Link and forgot which value you saved.
  - First-time bring-up: confirm the motor is alive without knowing its ID.

Prerequisites:
  - Motor powered (18-52 V)
  - CAN H/L wired, 120 Ohm termination present
  - R-Link CAN Mode = "MIT"  (Periodic Feedback / Servo mode will NOT
    respond to MIT polls -- that is a known limitation of cubemars_bus.py)
  - HDSC adapter on /dev/ttyACM0

Usage:
    python 00_scan_can_id.py                      # scan 0x01..0x80 (default)
    python 00_scan_can_id.py --start 1 --end 127  # custom range
    python 00_scan_can_id.py --port /dev/ttyACM1  # different adapter

If nothing responds in 0x01..0x7F, the motor is either:
  - In Servo mode (Periodic Feedback) -> change to MIT in R-Link
  - Unpowered, miswired, or has no 120 Ohm termination
  - Using extended-CAN IDs beyond 0x7F (rare for AK-series MIT mode)
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from DM_CAN import float_to_uint, uint_to_float  # noqa: E402
from motor_config import CUBEMARS_LIMITS         # noqa: E402

# HDSC USB-to-CAN 30-byte transmit envelope (same as cubemars_bus._SEND_FRAME)
_SEND_FRAME = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)

_FRAME_LEN = 16


def _build_null_mit_payload(model: str = "AK60-6") -> bytes:
    """Pack the safest MIT poll: q=0, dq=0, kp=0, kd=0, tau=0."""
    pmax, vmax, tmax = CUBEMARS_LIMITS[model]
    q_u   = float_to_uint(0.0, -pmax,  pmax,  16)
    dq_u  = float_to_uint(0.0, -vmax,  vmax,  12)
    kp_u  = float_to_uint(0.0,  0.0,   500.0, 12)
    kd_u  = float_to_uint(0.0,  0.0,   5.0,   12)
    tau_u = float_to_uint(0.0, -tmax,  tmax,  12)

    data = bytearray(8)
    data[0] = (q_u >> 8) & 0xFF
    data[1] =  q_u       & 0xFF
    data[2] =  dq_u >> 4
    data[3] = ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF)
    data[4] =  kp_u & 0xFF
    data[5] =  kd_u >> 4
    data[6] = ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF)
    data[7] =  tau_u & 0xFF
    return bytes(data)


def _send(ser: serial.Serial, can_id: int, payload: bytes) -> None:
    frame = _SEND_FRAME.copy()
    frame[13] =  can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[21:29] = np.frombuffer(payload, dtype=np.uint8)[:8]
    ser.write(bytes(frame))


def _extract_frames(data: bytes) -> tuple[list[bytes], bytes]:
    """Pull complete 16-byte HDSC feedback frames out of a raw byte stream."""
    frames: list[bytes] = []
    i = 0
    last = 0
    while i <= len(data) - _FRAME_LEN:
        if data[i] == 0xAA and data[i + _FRAME_LEN - 1] == 0x55:
            frames.append(data[i:i + _FRAME_LEN])
            i += _FRAME_LEN
            last = i
        else:
            i += 1
    return frames, data[last:]


def _decode_feedback(payload: bytes, model: str = "AK60-6"
                     ) -> tuple[float, float, float]:
    pmax, vmax, tmax = CUBEMARS_LIMITS[model]
    pos_u = np.uint16((payload[1] << 8) | payload[2])
    vel_u = np.uint16((payload[3] << 4) | (payload[4] >> 4))
    tau_u = np.uint16(((payload[4] & 0xF) << 8) | payload[5])
    q   = uint_to_float(pos_u, -pmax,  pmax,  16)
    dq  = uint_to_float(vel_u, -vmax,  vmax,  12)
    tau = uint_to_float(tau_u, -tmax,  tmax,  12)
    return float(q), float(dq), float(tau)


def scan(port: str, start: int, end: int, model: str,
         per_id_delay_s: float = 0.015) -> list[tuple[int, int, bytes]]:
    """Send a null MIT poll to every can_id in [start, end].

    Returns a list of (probed_id, replying_can_id, payload) tuples for every
    feedback frame received.  An ID is considered "alive" if any reply
    arrives within per_id_delay_s.
    """
    payload = _build_null_mit_payload(model)
    hits: list[tuple[int, int, bytes]] = []

    with serial.Serial(port, 921600, timeout=0.05) as ser:
        # Drain anything stale from the adapter before starting
        time.sleep(0.05)
        ser.read_all()

        print(f"Scanning CAN IDs 0x{start:02X}..0x{end:02X} on {port}")
        print(f"Model={model}  payload={payload.hex()}  (Kp=Kd=tau=0, q_des=0)\n")

        buf = b""
        for can_id in range(start, end + 1):
            _send(ser, can_id, payload)
            time.sleep(per_id_delay_s)

            raw = buf + ser.read_all()
            frames, buf = _extract_frames(raw)
            for fr in frames:
                if fr[1] != 0x11:
                    continue
                reply_id = (fr[6] << 24) | (fr[5] << 16) | (fr[4] << 8) | fr[3]
                hits.append((can_id, reply_id, bytes(fr[7:15])))

            # Live progress every 16 IDs
            if (can_id - start) % 16 == 15:
                print(f"  probed up to 0x{can_id:02X} ({can_id})  -- "
                      f"{len(hits)} hits so far")

        # Final drain in case last few frames arrived after the loop
        time.sleep(0.1)
        raw = buf + ser.read_all()
        frames, _ = _extract_frames(raw)
        for fr in frames:
            if fr[1] != 0x11:
                continue
            reply_id = (fr[6] << 24) | (fr[5] << 16) | (fr[4] << 8) | fr[3]
            hits.append((-1, reply_id, bytes(fr[7:15])))   # late reply

    return hits


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port",  default="/dev/ttyACM0")
    p.add_argument("--start", type=lambda x: int(x, 0), default=0x01)
    p.add_argument("--end",   type=lambda x: int(x, 0), default=0x80)
    p.add_argument("--model", default="AK60-6", choices=list(CUBEMARS_LIMITS))
    args = p.parse_args()

    hits = scan(args.port, args.start, args.end, args.model)

    print("\n" + "=" * 60)
    if not hits:
        print("No replies in the scanned range.")
        print("\nThe motor is probably in Servo mode (R-Link 'CAN Mode' =")
        print("'Periodic Feedback').  Change it to 'MIT' in R-Link, click")
        print("Write, power-cycle the motor, and rerun this scan.")
        print("\nAlso verify: power, CAN H/L polarity, 120 Ohm termination.")
        return 1

    # Group hits by the CAN ID that actually appeared in feedback
    by_reply: dict[int, list[int]] = {}
    for probed, reply_id, _payload in hits:
        by_reply.setdefault(reply_id, []).append(probed)

    print(f"Got {len(hits)} feedback frames from {len(by_reply)} CAN ID(s):\n")
    for reply_id, probes in sorted(by_reply.items()):
        probes_str = ", ".join(f"0x{p:02X}({p})" if p >= 0 else "<late>"
                               for p in probes[:6])
        more = f" +{len(probes)-6} more" if len(probes) > 6 else ""

        # Servo-mode (Periodic Feedback) extended CAN ID format:
        #   ext_id = (cmd_type << 8) | motor_id
        # cmd_type 0x29 = COMM_FOC_GET_VALUES (status frame 1, the common one)
        servo_cmd  = (reply_id >> 8) & 0xFF
        servo_mid  =  reply_id       & 0xFF
        servo_hint = ""
        if reply_id > 0xFF:
            servo_hint = (f"  [servo mode: cmd=0x{servo_cmd:02X}, "
                          f"motor_id=0x{servo_mid:02X}={servo_mid}]")

        # Try MIT decode only when the reply ID looks like a plain ESC_ID
        if reply_id <= 0x7F:
            try:
                q, dq, tau = _decode_feedback(
                    next(p for pr, rid, p in hits if rid == reply_id),
                    args.model)
                decoded = (f"MIT decode: q={q:+7.3f} rad  "
                           f"dq={dq:+7.3f} rad/s  tau={tau:+6.3f} N.m")
            except Exception:
                decoded = "(MIT decode failed)"
        else:
            decoded = "(extended-ID frame -- not MIT)"

        print(f"  Feedback CAN ID 0x{reply_id:04X} ({reply_id}){servo_hint}")
        print(f"    Replied to probes: {probes_str}{more}")
        print(f"    Sample           : {decoded}")
        print()

    # Best-guess winner
    winner = max(by_reply, key=lambda k: len(by_reply[k]))
    print("=" * 60)
    if winner > 0xFF:
        mid = winner & 0xFF
        print(f"Motor is in SERVO / Periodic Feedback mode.")
        print(f"Motor CAN ID = 0x{mid:02X} ({mid} decimal)")
        print(f"\nThis driver (cubemars_bus.py) speaks MIT mode only.")
        print(f"To use it: open R-Link, set CAN Mode = MIT, click Write,")
        print(f"power-cycle the motor, then rerun this scan.")
        print(f"\nIf you want to keep Servo mode, a servo-protocol driver")
        print(f"is needed (extended CAN frames, separate command set).")
    else:
        print(f"Most-likely motor CAN ID: 0x{winner:02X} ({winner})")
        print(f"\nUpdate src/motor_config.py:")
        print(f"  CubeMarsMotorConfig(can_id=0x{winner:02X}, "
              f"model=\"{args.model}\")")
    return 0


if __name__ == "__main__":
    sys.exit(main())
