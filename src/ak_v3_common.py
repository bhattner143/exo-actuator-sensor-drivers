"""ak_v3_common.py -- Shared constants and helpers for V3.0 protocol.

Used by both ``ak_v3_can.py`` (SocketCAN) and ``ak_v3_serial.py`` (HDSC).
No dependencies on python-can or serial -- just dataclasses and math.
"""
from __future__ import annotations
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# V3.0 packet-type IDs (manual §4.1 enum CAN_PACKET_ID)
# ---------------------------------------------------------------------------
CAN_PACKET_SET_DUTY          = 0
CAN_PACKET_SET_CURRENT       = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM           = 3
CAN_PACKET_SET_POS           = 4
CAN_PACKET_SET_ORIGIN_HERE   = 5
CAN_PACKET_SET_POS_SPD       = 6
CAN_PACKET_SET_MIT           = 8

# Feedback packet type lives in the upper bits of the extended reply ID.
# Empirically 0x29 (= packet type 41) for V3.0 ``Periodic Feedback``.
FEEDBACK_PACKET_TYPE = 0x29


# ---------------------------------------------------------------------------
# Per-model parameter ranges (manual §4.2 "Parameter Ranges" table)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class AkV3Limits:
    """MIT bit-packing limits for one motor model (manual §4.2 table)."""
    p_min: float = -12.56
    p_max: float =  12.56
    v_min: float = -45.0
    v_max: float =  45.0
    t_min: float = -18.0
    t_max: float =  18.0
    kp_max: float = 500.0
    kd_max: float =   5.0


# Verified from the manual Parameter Ranges table on p.42.
AK_V3_LIMITS: dict[str, AkV3Limits] = {
    "AK10-9":  AkV3Limits(v_min=-28.0,  v_max=28.0,  t_min=-54.0,  t_max=54.0),
    "AK60-6":  AkV3Limits(v_min=-60.0,  v_max=60.0,  t_min=-12.0,  t_max=12.0),
    "AK70-9":  AkV3Limits(v_min=-30.0,  v_max=30.0,  t_min=-32.0,  t_max=32.0),
    "AK80-9":  AkV3Limits(v_min=-65.0,  v_max=65.0,  t_min=-18.0,  t_max=18.0),
    "AKE60-8": AkV3Limits(v_min=-40.0,  v_max=40.0,  t_min=-15.0,  t_max=15.0),
    "AKE80-8": AkV3Limits(v_min=-20.0,  v_max=20.0,  t_min=-35.0,  t_max=35.0),
}


# ---------------------------------------------------------------------------
# Bit-packing helpers (verbatim port of ``float_to_uint`` from AK-V3.ino)
# ---------------------------------------------------------------------------
def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return x * span / ((1 << bits) - 1) + x_min
