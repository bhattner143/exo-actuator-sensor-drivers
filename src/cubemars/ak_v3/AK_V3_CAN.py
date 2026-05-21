"""AK_V3_CAN.py -- Python equivalent of ``AK-V3.ino`` using python-can.

Targets CubeMars AK-series **V3.0 driver-board firmware** as documented in
``AK Series Module Product Manual V3.0.1``.

Supports two CAN adapters:

* **gs_usb / candleLight firmware** (DSD TECH SH-C30A, Canable, Waveshare
  USB-CAN-A): USB ID ``1d50:606f``.  Uses python-can ``gs_usb`` backend via
  libusb -- **no gs_usb kernel module required**::

      bus = open_gs_usb_bus(bitrate=1_000_000)   # recommended on Jetson

* **SocketCAN** (Jetson built-in mttcan or any SocketCAN interface)::

      ensure_can0_up(1_000_000)
      bus = can.interface.Bus(channel="can0", bustype="socketcan")

V3.0 firmware protocol summary
------------------------------
All control modes (servo *and* force-control/MIT) use **CAN 2.0B extended
frames** where the 29-bit arbitration ID encodes both the packet type and
the driver ESC_ID::

    CAN_ID = (PACKET_TYPE << 8) | ESC_ID

Packet types (manual section 4.1, 4.2)::

    0  SET_DUTY                     7  -- (reserved)
    1  SET_CURRENT                  8  SET_MIT (force control)
    2  SET_CURRENT_BRAKE
    3  SET_RPM
    4  SET_POS
    5  SET_ORIGIN_HERE
    6  SET_POS_SPD

MIT (packet 8) byte layout in V3.0 is **Kp first**, NOT the classic
mini-cheetah ``q``-first layout used by Damiao and earlier T-Motor firmware::

    buffer[0] = kp_int >> 4                          # Kp high 8
    buffer[1] = ((kp & 0xF) << 4) | (kd_int >> 8)    # Kp low 4 | Kd high 4
    buffer[2] = kd_int & 0xFF                        # Kd low 8
    buffer[3] = p_int >> 8                           # pos high 8
    buffer[4] = p_int & 0xFF                         # pos low 8
    buffer[5] = v_int >> 4                           # vel high 8
    buffer[6] = ((v & 0xF) << 4) | (t_int >> 8)      # vel low 4 | tau high 4
    buffer[7] = t_int & 0xFF                         # tau low 8

Feedback frame on CAN ID 0x29XX (extended), 8 bytes::

    [0:1]  pos_int16     -> degrees  * 0.1
    [2:3]  spd_int16     -> ERPM     * 10
    [4:5]  cur_int16     -> amps     * 0.01
    [6]    temp_mos      -> int8 °C
    [7]    error_code    -> uint8

Bring up the bus once before importing (or call ``ensure_can0_up()``)::

    sudo ip link set can0 up type can bitrate 1000000

Reference: ``AK-V3.ino`` (CubeMars Arduino example) and AK V3.0.1 manual.
"""
from __future__ import annotations

import os
import struct
import time
from typing import Optional

import can

# Import shared V3.0 constants and helpers (no python-can dependency).
from .ak_v3_common import (
    CUBEMARS_AK_V3_LIMITS,
    CubeMarsAkV3Limits,
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
    uint_to_float,
)


# ---------------------------------------------------------------------------
# Bus management
# ---------------------------------------------------------------------------
def ensure_can0_up(bitrate: int = 1_000_000, channel: str = "can0") -> None:
    """Bring a SocketCAN interface up at the given bitrate (idempotent).

    Only needed when using the Jetson built-in mttcan (``can0``) or another
    SocketCAN interface.  Not required for the gs_usb backend.
    """
    os.system(f"sudo ip link set {channel} down 2>/dev/null")
    os.system(f"sudo ip link set {channel} up type can bitrate {bitrate}")


def open_gs_usb_bus(bitrate: int = 1_000_000, index: int = 0) -> can.BusABC:
    """Open the first gs_usb / candleLight device (e.g. DSD TECH SH-C30A).

    Uses python-can's ``gs_usb`` backend which communicates directly via
    libusb -- no ``gs_usb`` kernel module is required on the host system.

    Args:
        bitrate: CAN bus bitrate in bits/s (default 1 Mbps).
        index:   Which gs_usb device to open if multiple are attached.

    Returns:
        An open ``can.BusABC`` ready for use with ``CubeMarsAkV3Motor``.
    """
    return can.Bus(interface="gs_usb", channel=index, bitrate=bitrate, index=index)


class CubeMarsAkV3Motor:
    """Single CubeMars AK-series motor (V3.0 firmware) on a SocketCAN bus.

    Mirrors the structure of ``AK-V3.ino`` -- one ``MCP2515 mcp2515(9)``
    object is replaced by a shared ``can.Bus`` injected at construction.
    """

    def __init__(
        self,
        bus: can.BusABC,
        can_id: int,
        model: str = "AK60-6",
    ) -> None:
        if model not in CUBEMARS_AK_V3_LIMITS:
            raise ValueError(f"Unknown model {model!r}; known: {list(CUBEMARS_AK_V3_LIMITS)}")
        self.bus = bus
        self.can_id = can_id           # ESC_ID, e.g. 104 (0x68) -- in DECIMAL.
        self.model = model
        self.limits = CUBEMARS_AK_V3_LIMITS[model]

        # Latest decoded feedback (output-shaft units).
        self.pos_deg: float = 0.0
        self.spd_erpm: float = 0.0
        self.current_a: float = 0.0
        self.temp_c: int = 0
        self.error: int = 0

    # --- low-level frame transmission -----------------------------------
    def _send_eid(self, packet_type: int, payload: bytes) -> None:
        """Send extended-ID frame with ``CAN_ID = (packet_type << 8) | ESC_ID``."""
        ext_id = (packet_type << 8) | self.can_id
        msg = can.Message(
            arbitration_id=ext_id,
            data=payload,
            is_extended_id=True,
        )
        self.bus.send(msg)

    # --- Servo-mode commands (manual §4.1) ------------------------------
    def set_duty(self, duty: float) -> None:
        """Duty-cycle mode. ``duty`` in [-1, 1]."""
        val = int(duty * 100_000.0)
        self._send_eid(CAN_PACKET_SET_DUTY, struct.pack(">i", val))

    def set_current(self, amps: float) -> None:
        """Current-loop mode (Iq, amps). Torque = Iq * Kt."""
        val = int(amps * 1_000.0)
        self._send_eid(CAN_PACKET_SET_CURRENT, struct.pack(">i", val))

    def set_brake_current(self, amps: float) -> None:
        """Current-brake mode; holds rotor with the given braking current."""
        val = int(amps * 1_000.0)
        self._send_eid(CAN_PACKET_SET_CURRENT_BRAKE, struct.pack(">i", val))

    def set_rpm(self, erpm: float) -> None:
        """Velocity-loop mode. Note ERPM = mechanical_RPM * pole_pairs."""
        val = int(erpm)
        self._send_eid(CAN_PACKET_SET_RPM, struct.pack(">i", val))

    def set_position_deg(self, degrees: float) -> None:
        """Position-loop mode. ``degrees`` in [-36000, 36000]."""
        val = int(degrees * 10_000.0)
        self._send_eid(CAN_PACKET_SET_POS, struct.pack(">i", val))

    def set_origin(self, permanent: bool = False) -> None:
        """Latch current rotor angle as zero.

        ``permanent=False`` -> erased on power loss.
        ``permanent=True``  -> written to flash (irreversible, do not spam).
        """
        self._send_eid(CAN_PACKET_SET_ORIGIN_HERE, bytes([1 if permanent else 0]))

    def set_pos_spd(self, degrees: float, erpm: int, acc_erpm_s2: int) -> None:
        """Trapezoidal position-velocity-acceleration profile."""
        pos = int(degrees * 10_000.0)
        spd = int(erpm // 10)
        acc = int(acc_erpm_s2 // 10)
        payload = struct.pack(">i", pos) + struct.pack(">h", spd) + struct.pack(">h", acc)
        self._send_eid(CAN_PACKET_SET_POS_SPD, payload)

    # --- Force-control (MIT) command  (manual §4.2) ---------------------
    def set_mit(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
    ) -> None:
        """V3.0 MIT frame (Kp-first byte order, extended ID = 0x800 | ESC_ID).

        Sub-modes are selected purely by which arguments are nonzero:
          - **Position** :  kp > 0, kd > 0, set p_des.  Never kd=0.
          - **Velocity** :  kp = 0, kd > 0, set v_des.
          - **Torque**   :  kp = 0, kd = 0, set t_ff.   (open loop)
        """
        L = self.limits
        p_int  = float_to_uint(p_des, L.p_min,  L.p_max, 16)
        v_int  = float_to_uint(v_des, L.v_min,  L.v_max, 12)
        kp_int = float_to_uint(kp,     0.0,     L.kp_max, 12)
        kd_int = float_to_uint(kd,     0.0,     L.kd_max, 12)
        t_int  = float_to_uint(t_ff,  L.t_min,  L.t_max, 12)

        # NOTE: byte order matches AK-V3.ino pack_cmd() (Kp first).
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

    # --- Feedback decoding ----------------------------------------------
    def parse_feedback(self, msg: can.Message) -> bool:
        """Decode an incoming servo-style feedback frame.

        Returns True if the frame was addressed to this motor.
        Expected ``msg.arbitration_id == (FEEDBACK_PACKET_TYPE << 8) | ESC_ID``.
        """
        if not msg.is_extended_id:
            return False
        if msg.arbitration_id != (FEEDBACK_PACKET_TYPE << 8) | self.can_id:
            return False
        d = msg.data
        pos_int = int.from_bytes(d[0:2], "big", signed=True)
        spd_int = int.from_bytes(d[2:4], "big", signed=True)
        cur_int = int.from_bytes(d[4:6], "big", signed=True)
        self.pos_deg   = pos_int * 0.1
        self.spd_erpm  = spd_int * 10.0
        self.current_a = cur_int * 0.01
        self.temp_c    = int.from_bytes(d[6:7], "big", signed=True)
        self.error     = d[7]
        return True

    def poll_feedback(self, timeout: float = 0.05) -> bool:
        """Block until one feedback frame for this motor arrives."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            # Minimum 10 ms chunk so recv() never gets timeout=0
            # (gs_usb backend blocks forever on timeout=0).
            msg = self.bus.recv(timeout=max(0.01, min(remaining, 0.05)))
            if msg is None:
                continue
            if self.parse_feedback(msg):
                return True


# ---------------------------------------------------------------------------
# Convenience top-level demo (mirrors the AK-V3.ino loop())
# ---------------------------------------------------------------------------
def _demo() -> None:
    # Use gs_usb backend (DSD TECH SH-C30A or Canable) -- no kernel module needed.
    # To use Jetson built-in CAN instead:
    #   ensure_can0_up(1_000_000)
    #   bus = can.interface.Bus(channel="can0", bustype="socketcan")
    bus = open_gs_usb_bus(bitrate=1_000_000)
    motor = CubeMarsAkV3Motor(bus, can_id=104, model="AK60-6")     # 104 == 0x68

    # 1 N.m open-loop torque pulse for 100 ms, mirroring the .ino default:
    print("Sending MIT torque = 1 N.m (open loop)")
    motor.set_mit(p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=1.0)

    for _ in range(20):
        if motor.poll_feedback(timeout=0.05):
            print(
                f"pos={motor.pos_deg:+7.2f} deg  "
                f"spd={motor.spd_erpm:+7.1f} ERPM  "
                f"I={motor.current_a:+5.2f} A  "
                f"T={motor.temp_c:3d} C  err={motor.error}"
            )
    bus.shutdown()


if __name__ == "__main__":
    _demo()
