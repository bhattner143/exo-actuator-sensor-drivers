"""ak_v3_serial.py -- CubeMars AK V3.0 over the Damiao HDSC USB-CAN adapter.

EXPERIMENTAL.  Combines:
  * the **HDSC 30-byte serial envelope** (same transport as ``cubemars_bus.py``
    and ``DM_CAN.py``); and
  * the **V3.0 firmware protocol** (extended 29-bit CAN IDs, Kp-first MIT
    byte order, plain-int16 feedback) -- same wire format as
    ``ak_v3_can.py``.

Why this exists
---------------
The user does not have a SocketCAN-capable adapter (Canable / Waveshare
USB-CAN-A) yet, only the proprietary HDSC adapter that ships with
Damiao motors.  ``DM_CAN.py`` documents byte 8 of the envelope as::

    bit3 = 0  -> standard (11-bit) CAN ID    (default, value 0x0a)
    bit3 = 1  -> extended (29-bit) CAN ID    (value 0x1a)

The receive side already parses a 4-byte CAN ID, so it can naturally
deliver extended IDs back to the host.  If the adapter firmware honours
the bit-3 flag, this driver can talk to a V3.0 board using the existing
HDSC dongle.

Whether the HDSC firmware actually does honour bit 3 is **undetermined**
-- the documentation table in ``DM_CAN.py`` is inferred from the Damiao
USB protocol notes, not from CubeMars docs.  The motor will simply not
respond if extended-ID transmit is unsupported, with no harm done.

Usage
-----
    from ak_v3_serial import AkV3Serial

    with AkV3Serial(port="/dev/ttyACM0", can_id=0x02, model="AK60-6") as m:
        m.set_mit(p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.5)
        if m.poll_feedback(timeout=0.1):
            print(m.pos_deg, m.spd_erpm, m.current_a)
"""
from __future__ import annotations

import time
import struct
from dataclasses import dataclass
from typing import Optional

import numpy as np
import serial

# Re-use V3.0 protocol constants and helpers (no python-can dependency).
from ak_v3_common import (
    AK_V3_LIMITS,
    AkV3Limits,
    CAN_PACKET_SET_DUTY,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD,
    CAN_PACKET_SET_MIT,
    FEEDBACK_PACKET_TYPE,
    float_to_uint,
)


# HDSC 30-byte transmit envelope (template).  See DM_CAN.py header comment
# for the field-by-field layout.  Key differences for extended IDs:
#   - byte [8]   : DLC + flags.  Set bit 3 to request a 29-bit ext frame.
#   - bytes [13..16] : 4-byte little-endian CAN ID (used for both std & ext).
_SEND_FRAME_STD = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00], dtype=np.uint8)

# Extended-frame variant: bit 3 of byte [8] set => IDE bit on the wire.
# Some Damiao firmware revisions use 0x1a (0x0a | 0x10) instead of (0x0a | 0x08);
# we expose both for empirical testing.
_BYTE8_EXT_PRIMARY   = 0x0a | 0x08    # 0x12: documented in DM_CAN.py comment
_BYTE8_EXT_FALLBACK  = 0x0a | 0x10    # 0x1a: alternate -- some firmware uses bit4

_FRAME_LEN_RECV = 16   # [0xAA, CMD, DLC, CAN_ID*4, payload*8, 0x55]

# V3.0 has no enable/disable/zero magic frames; ``set_origin`` covers zero.
# Brake-current = 0 acts as a soft stop for the servo modes.


class AkV3Serial:
    """CubeMars AK V3.0 driver-board over the HDSC USB-CAN serial adapter.

    Mirrors the surface of ``AkV3Motor`` from ``ak_v3_can.py`` so test
    scripts can switch transports by changing only the import line.
    """

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 921600,
        can_id: int = 0x02,
        model: str = "AK60-6",
        ext_flag_byte: int = _BYTE8_EXT_PRIMARY,
    ) -> None:
        if model not in AK_V3_LIMITS:
            raise ValueError(f"Unknown model {model!r}; known: {list(AK_V3_LIMITS)}")
        self.port = port
        self.baudrate = baudrate
        self.can_id = can_id
        self.model = model
        self.limits = AK_V3_LIMITS[model]
        self._ext_flag = ext_flag_byte
        self._ser: serial.Serial | None = None
        self._buf: bytes = b""

        # Latest decoded feedback (output-shaft units).
        self.pos_deg: float = 0.0
        self.spd_erpm: float = 0.0
        self.current_a: float = 0.0
        self.temp_c: int = 0
        self.error: int = 0

    # --- lifecycle ------------------------------------------------------
    def connect(self) -> None:
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        print(f"[AkV3Serial] opened {self.port} @ {self.baudrate}")

    def disconnect(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def __enter__(self) -> "AkV3Serial":
        self.connect()
        return self

    def __exit__(self, *_exc) -> None:
        self.disconnect()

    # --- low-level frame transmission ----------------------------------
    def _send_eid(self, packet_type: int, payload: bytes) -> None:
        """Send extended-ID frame via HDSC envelope.

        CAN_ID = (packet_type << 8) | ESC_ID   (29-bit extended).
        """
        if self._ser is None:
            raise RuntimeError("AkV3Serial not connected. Call connect() first.")
        ext_id = (packet_type << 8) | self.can_id

        frame = _SEND_FRAME_STD.copy()
        # Mark the frame as extended (29-bit) -- bit 3 of byte 8.
        frame[8] = self._ext_flag
        # 4-byte little-endian CAN ID at offsets 13..16.
        frame[13] =  ext_id        & 0xFF
        frame[14] = (ext_id >>  8) & 0xFF
        frame[15] = (ext_id >> 16) & 0xFF
        frame[16] = (ext_id >> 24) & 0xFF

        # Right-pad payload to 8 bytes (servo-mode payloads are 4 or 8 bytes).
        if len(payload) < 8:
            payload = payload + b"\x00" * (8 - len(payload))
        frame[21:29] = np.frombuffer(payload[:8], dtype=np.uint8)
        self._ser.write(bytes(frame))

    # --- Servo-mode commands (manual §4.1) ------------------------------
    def set_duty(self, duty: float) -> None:
        val = int(duty * 100_000.0)
        self._send_eid(CAN_PACKET_SET_DUTY, struct.pack(">i", val))

    def set_current(self, amps: float) -> None:
        val = int(amps * 1_000.0)
        self._send_eid(CAN_PACKET_SET_CURRENT, struct.pack(">i", val))

    def set_brake_current(self, amps: float) -> None:
        val = int(amps * 1_000.0)
        self._send_eid(CAN_PACKET_SET_CURRENT_BRAKE, struct.pack(">i", val))

    def set_rpm(self, erpm: float) -> None:
        self._send_eid(CAN_PACKET_SET_RPM, struct.pack(">i", int(erpm)))

    def set_position_deg(self, degrees: float) -> None:
        val = int(degrees * 10_000.0)
        self._send_eid(CAN_PACKET_SET_POS, struct.pack(">i", val))

    def set_origin(self, permanent: bool = False) -> None:
        self._send_eid(CAN_PACKET_SET_ORIGIN_HERE,
                       bytes([1 if permanent else 0]))

    def set_pos_spd(self, degrees: float, erpm: int, acc_erpm_s2: int) -> None:
        pos = int(degrees * 10_000.0)
        spd = int(erpm // 10)
        acc = int(acc_erpm_s2 // 10)
        payload = struct.pack(">i", pos) + struct.pack(">h", spd) + struct.pack(">h", acc)
        self._send_eid(CAN_PACKET_SET_POS_SPD, payload)

    # --- Force-control (MIT) command -----------------------------------
    def set_mit(
        self,
        p_des: float = 0.0,
        v_des: float = 0.0,
        kp: float    = 0.0,
        kd: float    = 0.0,
        t_ff: float  = 0.0,
    ) -> None:
        """V3.0 MIT frame -- Kp-first byte order, extended ID = (8<<8)|ESC."""
        L = self.limits
        p_int  = float_to_uint(p_des, L.p_min,  L.p_max, 16)
        v_int  = float_to_uint(v_des, L.v_min,  L.v_max, 12)
        kp_int = float_to_uint(kp,     0.0,     L.kp_max, 12)
        kd_int = float_to_uint(kd,     0.0,     L.kd_max, 12)
        t_int  = float_to_uint(t_ff,  L.t_min,  L.t_max, 12)
        buf = bytes([
            (kp_int >> 4) & 0xFF,
            ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF),
            kd_int & 0xFF,
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
            t_int & 0xFF,
        ])
        self._send_eid(CAN_PACKET_SET_MIT, buf)

    # --- Feedback reception --------------------------------------------
    def _read_frames(self) -> list[bytes]:
        if self._ser is None:
            return []
        raw = self._buf + self._ser.read_all()
        frames: list[bytes] = []
        i = 0
        last = 0
        while i <= len(raw) - _FRAME_LEN_RECV:
            if raw[i] == 0xAA and raw[i + _FRAME_LEN_RECV - 1] == 0x55:
                frames.append(raw[i:i + _FRAME_LEN_RECV])
                i += _FRAME_LEN_RECV
                last = i
            else:
                i += 1
        self._buf = raw[last:]
        return frames

    def _parse_one(self, frame: bytes) -> bool:
        """Try to decode a single HDSC feedback frame as a V3.0 reply."""
        if frame[1] != 0x11:           # not a CAN-frame delivery
            return False
        can_id = ((frame[6] << 24) | (frame[5] << 16)
                  | (frame[4] << 8)  |  frame[3])
        expected = (FEEDBACK_PACKET_TYPE << 8) | self.can_id
        # HDSC may also encode the IDE bit somewhere -- compare on lower 29 bits.
        if (can_id & 0x1FFFFFFF) != (expected & 0x1FFFFFFF):
            return False
        d = frame[7:15]
        pos_int = int.from_bytes(d[0:2], "big", signed=True)
        spd_int = int.from_bytes(d[2:4], "big", signed=True)
        cur_int = int.from_bytes(d[4:6], "big", signed=True)
        self.pos_deg   = pos_int * 0.1
        self.spd_erpm  = spd_int * 10.0
        self.current_a = cur_int * 0.01
        self.temp_c    = int.from_bytes(d[6:7], "big", signed=True)
        self.error     = d[7]
        return True

    def poll_feedback(self, timeout: float = 0.1) -> bool:
        """Block until one V3.0 feedback frame for this motor arrives."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for frame in self._read_frames():
                if self._parse_one(frame):
                    return True
            time.sleep(0.002)
        return False

    # --- Debug helper ---------------------------------------------------
    def dump_raw(self, duration: float = 1.0) -> list[bytes]:
        """Read raw serial bytes for ``duration`` seconds; return frame list.

        Useful to confirm anything at all is coming back from the adapter.
        """
        deadline = time.monotonic() + duration
        all_frames: list[bytes] = []
        while time.monotonic() < deadline:
            all_frames.extend(self._read_frames())
            time.sleep(0.01)
        return all_frames


# ---------------------------------------------------------------------------
# Smoke test (mirrors ak_v3_can.py _demo)
# ---------------------------------------------------------------------------
def _demo() -> None:
    with AkV3Serial(port="/dev/ttyACM0", can_id=0x02, model="AK60-6") as m:
        print("Sending MIT zero-torque ping...")
        m.set_mit()                       # all zeros -- just elicit feedback
        time.sleep(0.05)
        frames = m.dump_raw(duration=0.5)
        print(f"received {len(frames)} HDSC frames")
        for f in frames[:5]:
            print(" ", f.hex())
        if m.poll_feedback(timeout=0.5):
            print(f"pos={m.pos_deg:+7.2f} deg  spd={m.spd_erpm:+7.1f} ERPM  "
                  f"I={m.current_a:+5.2f} A  T={m.temp_c} C  err={m.error}")
        else:
            print("No V3.0-formatted feedback parsed.")


if __name__ == "__main__":
    _demo()
