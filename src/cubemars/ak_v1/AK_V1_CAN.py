"""AK_V1_CAN.py -- SocketCAN driver for CubeMars AK-series V1.x firmware.

Targets older CubeMars driver boards running firmware V1.x (pre-V3.0),
as documented in *AK Series Module Driver User Manual V1.0.15.X*.

This covers the **AK80-8 KV60** shoulder motor on this bench.

Key differences from V3.0 (AK_V3_CAN.py)
------------------------------------------
- Uses **CAN 2.0A standard frames** (11-bit arbitration ID = ESC_ID).
  V3.0 uses 29-bit extended frames.
- MIT byte layout is **position-first** (same as mini-cheetah / Damiao).
  V3.0 uses Kp-first layout.
- Motor must be explicitly put into MIT mode with a special 8-byte frame
  before any MIT commands are accepted.
- Feedback frame also uses the same 11-bit ID with DATA[0] = motor ID,
  making the feedback distinguishable from host-sent frames.

MIT mode byte layout (manual §5.3, page 41/43)
-----------------------------------------------
Command (host → motor):
  data[0] = p_int >> 8                               # pos  high 8
  data[1] = p_int & 0xFF                             # pos  low  8
  data[2] = v_int >> 4                               # vel  high 8
  data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)    # vel  low  4 | Kp  high 4
  data[4] = kp_int & 0xFF                            # Kp   low  8
  data[5] = kd_int >> 4                              # Kd   high 8
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)    # Kd   low  4 | tau high 4
  data[7] = t_int & 0xFF                             # tau  low  8

Feedback (motor → host):
  data[0] = Driver ID                                (distinguishes from TX frames)
  data[1..2] = position (16-bit)
  data[3..4] = speed (12-bit, packed across nibble boundary)
  data[4..5] = current (12-bit, lower nibble of [4], all of [5])
  data[6]    = temperature (°C + 40 offset)
  data[7]    = error flag

Special 8-byte CAN frames (standard frame, ID = ESC_ID):
  Enter MIT mode : 0xFF × 7 + 0xFC
  Exit  MIT mode : 0xFF × 7 + 0xFD
  Set zero pos   : 0xFF × 7 + 0xFE

Parameter ranges (manual §5.3 table, AK80-8):
  Position : ±12.5 rad   Velocity : ±37.5 rad/s
  Torque   : ±32.0 N·m   Kp : 0–500   Kd : 0–5

Bench config:
  Motor  : AK80-8 KV60 (shoulder),  ESC_ID = 0x01
  Bus    : SocketCAN can1 (DSDTech SH-C30A, gs_usb module, 1 Mbps)
"""
from __future__ import annotations

import time
from typing import Optional

import can


# ---------------------------------------------------------------------------
# AK80-8 parameter ranges (manual §5.3 table, page 42)
# ---------------------------------------------------------------------------
P_MIN   = -12.5
P_MAX   =  12.5
V_MIN   = -37.5
V_MAX   =  37.5
T_MIN   = -32.0
T_MAX   =  32.0
KP_MAX  = 500.0
KD_MAX  =   5.0

# ---------------------------------------------------------------------------
# Bit-packing helpers (verbatim from manual pack_cmd / unpack_reply, p.43-44)
# ---------------------------------------------------------------------------

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Scale float in [x_min, x_max] to an unsigned integer of <bits> bits."""
    span = x_max - x_min
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """Scale unsigned integer of <bits> bits back to float in [x_min, x_max]."""
    span  = x_max - x_min
    return x * span / ((1 << bits) - 1) + x_min


# ---------------------------------------------------------------------------
# Motor driver
# ---------------------------------------------------------------------------

class CubeMarsAkV1Motor:
    """Single CubeMars AK-series V1.x motor on a SocketCAN bus.

    The motor must be switched to MIT mode via the R-Link tool before use,
    OR call ``enter_mode()`` which sends the 0xFF×7+0xFC special frame.

    Example::

        import can
        bus  = can.Bus(channel="can1", interface="socketcan")
        mot  = CubeMarsAkV1Motor(bus, can_id=0x01)
        mot.enter_mode()
        mot.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)      # zero-torque ping
        if mot.poll_feedback(timeout=0.2):
            print(mot.pos_rad, mot.vel_rad_s)
        mot.exit_mode()
        bus.shutdown()
    """

    def __init__(
        self,
        bus: can.BusABC,
        can_id: int = 0x01,
    ) -> None:
        self.bus    = bus
        self.can_id = can_id

        # Latest decoded feedback (SI units)
        self.pos_rad:   float = 0.0      # rad
        self.vel_rad_s: float = 0.0      # rad/s
        self.current_a: float = 0.0      # A (Iq, proportional to torque)
        self.temp_c:    int   = 0        # °C
        self.error:     int   = 0        # error flag byte

    # ---- private helper ------------------------------------------------

    def _send_std(self, data: bytes) -> None:
        """Send an 8-byte standard frame to the motor."""
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)

    # ---- mode control --------------------------------------------------

    def enter_mode(self) -> None:
        """Put the motor into MIT control mode (must call before MIT commands).

        Sends the special 'enter' frame: 0xFF×7 + 0xFC (manual §5.3, p.41).
        """
        self._send_std(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC')

    def exit_mode(self) -> None:
        """Exit MIT control mode.  Motor returns to idle (no torque output)."""
        self._send_std(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD')

    def set_zero(self) -> None:
        """Latch current shaft angle as zero (volatile, RAM only).

        Sends the special 'set zero' frame: 0xFF×7 + 0xFE (manual §5.3, p.41).
        """
        self._send_std(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE')

    # ---- MIT command ---------------------------------------------------

    def set_mit(
        self,
        p_des: float,
        v_des: float,
        kp:    float,
        kd:    float,
        t_ff:  float,
    ) -> None:
        """Send one MIT frame (position-first byte order).

        Sub-modes are selected by which arguments are non-zero:
          - **Position** : kp > 0, kd > 0, p_des = target.  Never kd=0.
          - **Velocity** : kp = 0, kd > 0, v_des = target.
          - **Torque**   : kp = 0, kd = 0, t_ff  = desired torque.

        Args:
            p_des : desired position (rad) in [P_MIN, P_MAX].
            v_des : desired velocity (rad/s) in [V_MIN, V_MAX].
            kp    : position stiffness [0, 500].
            kd    : velocity damping   [0, 5].  NEVER 0 in position mode.
            t_ff  : feed-forward torque (N·m) in [T_MIN, T_MAX].
        """
        p_int  = float_to_uint(p_des, P_MIN,  P_MAX,  16)
        v_int  = float_to_uint(v_des, V_MIN,  V_MAX,  12)
        kp_int = float_to_uint(kp,    0.0,    KP_MAX, 12)
        kd_int = float_to_uint(kd,    0.0,    KD_MAX, 12)
        t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12)

        data = bytes([
            (p_int  >> 8) & 0xFF,
            p_int  & 0xFF,
            (v_int  >> 4) & 0xFF,
            ((v_int  & 0xF) << 4) | ((kp_int >> 8) & 0xF),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0xF) << 4) | ((t_int  >> 8) & 0xF),
            t_int  & 0xFF,
        ])
        self._send_std(data)

    # ---- feedback ------------------------------------------------------

    def parse_feedback(self, msg: can.Message) -> bool:
        """Decode an incoming feedback frame from the motor.

        Returns True if the frame belongs to this motor.

        V1.x feedback (manual §5.3, p.42 + receive code p.44):
          data[0]   = Driver ID  (distinguishes from command frames)
          data[1:2] = position 16-bit
          data[3:4] = velocity  12-bit (high 8 in [3], high nibble of [4])
          data[4:5] = current   12-bit (low  nibble of [4], all of [5])
          data[6]   = temperature  (raw; Celsius = raw - 40, range -40..215)
          data[7]   = error flag
        """
        if msg.is_extended_id:
            return False
        if msg.arbitration_id != self.can_id:
            return False
        if len(msg.data) < 8:
            return False
        # data[0] holds the Driver ID in feedback frames.
        # Our own TX frames have data[0] = position high byte (0–0xFF),
        # which coincidentally could equal can_id.  The safest check is
        # that data[0] == can_id AND the bit pattern is consistent with a
        # 16-bit position at data[1:2] being non-trivially decodable.
        # For robustness we check data[0] == can_id only.
        if msg.data[0] != self.can_id:
            return False

        d = msg.data
        p_int = (d[1] << 8) | d[2]
        v_int = (d[3] << 4) | (d[4] >> 4)
        i_int = ((d[4] & 0xF) << 8) | d[5]

        self.pos_rad   = uint_to_float(p_int, P_MIN, P_MAX, 16)
        # Manual: speed stored in rad/s range matching V_MIN..V_MAX
        self.vel_rad_s = uint_to_float(v_int, V_MIN, V_MAX, 12)
        self.current_a = uint_to_float(i_int, -T_MAX, T_MAX, 12)  # Iq ≈ torque proxy
        self.temp_c    = int(d[6]) - 40         # manual: range -40..215 °C
        self.error     = int(d[7])
        return True

    def poll_feedback(self, timeout: float = 0.1) -> bool:
        """Block until one valid feedback frame arrives (or timeout).

        Returns True if a feedback frame was decoded before the deadline.
        """
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            msg = self.bus.recv(timeout=max(0.01, min(remaining, 0.05)))
            if msg is None:
                continue
            if self.parse_feedback(msg):
                return True
