"""motor_config.py -- Declarative motor and bus configuration dataclasses.

Follows the LeRobot pattern: hardware wiring (CAN IDs, serial ports, model
limits) is captured in plain dataclass instances that can be constructed
inline, loaded from YAML/JSON, or passed between modules without touching
any driver code.

Supported hardware
------------------
Damiao DM-J4310-2EC (and other DM4310-family motors)
    Protocol: MIT bit-packed CAN frame over HDSC USB-to-CAN (serial).
    Python driver: src/DM_CAN.py (unchanged vendor driver).

CubeMars AK series (AK10-9, AK60-6, AK70-10, AK80-6, AK80-9, AK80-64, ...)
    Protocol: identical MIT bit-packed CAN frame over HDSC USB-to-CAN.
    Feedback differs: CAN ID of feedback frame = motor's own ESC_ID (not MST_ID).

Quick-start example (Damiao + CubeMars V3.0 + CubeMars V1.x)
-------------------------------------------------------------
    from motor_config import (
        DamiaoBusConfig, DamiaoMotorConfig,
        CubeMarsAkV3BusConfig, CubeMarsMotorConfig,
        CubeMarsAkV1BusConfig, CubeMarsAkV1MotorConfig,
    )

    damiao_cfg = DamiaoBusConfig(
        port="/dev/ttyACM0",
        motors={
            "shoulder": DamiaoMotorConfig(can_id=0x01),
            "elbow":    DamiaoMotorConfig(can_id=0x02),
        },
    )
    ak60_cfg = CubeMarsAkV3BusConfig(
        channel="can1",
        motors={"wrist": CubeMarsMotorConfig(can_id=0x02, model="AK60-6")},
    )
    ak80_cfg = CubeMarsAkV1BusConfig(
        channel="can1",
        motors={"shoulder": CubeMarsAkV1MotorConfig(can_id=0x01, model="AK80-8")},
    )
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

# Ensure src/ is in sys.path for package imports (damiao.DM_CAN, etc.)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from damiao.DM_CAN import DM_Motor_Type

# ---------------------------------------------------------------------------
# CubeMars AK-series MIT limits  [PMAX (rad), VMAX (rad/s), TMAX (N·m)]
# ---------------------------------------------------------------------------
# Source: CubeMars datasheets + MIT-controller community tables.
# These are the values the firmware uses for bit-packing -- they define the
# full-scale range of the 16/12-bit position/velocity/torque fields.
CUBEMARS_LIMITS: dict[str, list[float]] = {
    "AK10-9":  [12.5,  50.0,  65.0],   # high-torque, low-speed planetary
    "AK60-6":  [12.5,  45.0,  15.0],   # medium-duty (V3.0: P=±12.56, V=±60, T=±12; bench uses firmware defaults)
    "AK70-10": [12.5,  50.0,  24.8],   # medium-duty, higher speed
    "AK80-6":  [12.5,  45.0,  15.0],   # same limits as AK60-6
    "AK80-8":  [12.5,  37.5,  32.0],   # V1.x -- manual §5.3 table (shoulder motor on this bench)
    "AK80-9":  [12.5,  50.0,  18.0],   # popular for bipeds / arms
    "AK80-64": [12.5,   8.0, 144.0],   # very high torque (64:1 gearbox)
    "AK10-9V2":[12.5,  50.0,  65.0],   # V2 hardware, same limits
}


# ---------------------------------------------------------------------------
# Per-motor configuration -- Damiao
# ---------------------------------------------------------------------------
@dataclass
class DamiaoMotorConfig:
    """Configuration for one Damiao motor on a CAN bus.

    Attributes
    ----------
    can_id    : ESC_ID programmed in firmware (default 0x01).
    master_id : MST_ID programmed in firmware.  MUST be 0x11 (not 0x00).
                With 0x00 the driver drops all feedback frames.
    model     : DM_Motor_Type enum member.  Determines PMAX/VMAX/TMAX used
                for MIT bit-packing (synced from firmware at connect time).
    """
    can_id:    int             = 0x01
    master_id: int             = 0x11          # MUST match firmware MST_ID
    model:     DM_Motor_Type   = DM_Motor_Type.DM4310


# ---------------------------------------------------------------------------
# Per-motor configuration -- CubeMars
# ---------------------------------------------------------------------------
@dataclass
class CubeMarsMotorConfig:
    """Configuration for one CubeMars AK-series motor on a CAN bus.

    Attributes
    ----------
    can_id : ESC_ID set on the motor (DIP switches or configuration tool).
    model  : String key into CUBEMARS_LIMITS, e.g. "AK60-6".
             Determines the PMAX/VMAX/TMAX scaling for the MIT frame.

    Note: CubeMars does not use a separate Master ID.  Feedback frames
    arrive with CAN ID = ESC_ID (not a host master address), so no
    master_id field is needed.
    """
    can_id: int = 0x01
    model:  str = "AK60-6"

    def __post_init__(self):
        if self.model not in CUBEMARS_LIMITS:
            raise ValueError(
                f"Unknown CubeMars model '{self.model}'. "
                f"Known models: {list(CUBEMARS_LIMITS)}"
            )


# ---------------------------------------------------------------------------
# Per-motor configuration -- CubeMars AK-series V1.x (standard CAN frames)
# ---------------------------------------------------------------------------
@dataclass
class CubeMarsAkV1MotorConfig:
    """Configuration for one CubeMars AK-series V1.x motor on SocketCAN.

    V1.x firmware uses standard 11-bit CAN frames (CAN 2.0A) and a
    position-first MIT byte layout.  Distinct from V3.0 which uses 29-bit
    extended frames.

    Attributes
    ----------
    can_id : ESC_ID programmed on the motor (DIP switches / R-Link).
    model  : String key into CUBEMARS_LIMITS, e.g. "AK80-8".
             Determines the PMAX/VMAX/TMAX scaling for the MIT frame.
    """
    can_id: int = 0x01
    model:  str = "AK80-8"

    def __post_init__(self):
        if self.model not in CUBEMARS_LIMITS:
            raise ValueError(
                f"Unknown AK V1.x model '{self.model}'. "
                f"Known models: {list(CUBEMARS_LIMITS)}"
            )


# ---------------------------------------------------------------------------
# Bus-level configuration -- Damiao
# ---------------------------------------------------------------------------
@dataclass
class DamiaoBusConfig:
    """Configuration for the DamiaoBus (one HDSC USB-to-CAN serial port).

    Attributes
    ----------
    port     : Serial device path, e.g. "/dev/ttyACM0" (Linux) or "COM3" (Windows).
    motors   : Mapping of human-readable name -> DamiaoMotorConfig.
               Names become the keys for read() and write() calls.
    baudrate : Serial baud rate of the HDSC adapter.  Do not change (921600 fixed).
    """
    port:     str
    motors:   dict[str, DamiaoMotorConfig] = field(default_factory=dict)
    baudrate: int = 921600


# ---------------------------------------------------------------------------
# Bus-level configuration -- CubeMars AK-series V3.0 (SocketCAN)
# ---------------------------------------------------------------------------
@dataclass
class CubeMarsAkV3BusConfig:
    """Configuration for CubeMarsAkV3Bench (CubeMars V3.0 firmware over SocketCAN).

    V3.0 uses CAN 2.0B extended frames (29-bit arbitration IDs) over a
    SocketCAN interface (e.g. can1 via DSDTech SH-C30A + gs_usb module).

    Attributes
    ----------
    channel : SocketCAN interface name, e.g. "can1".
    motors  : Mapping of human-readable name -> CubeMarsMotorConfig.
    bitrate : CAN bitrate in bps (default 1 Mbps; do not change).
    """
    channel: str
    motors:  dict[str, CubeMarsMotorConfig] = field(default_factory=dict)
    bitrate: int = 1_000_000


# ---------------------------------------------------------------------------
# Bus-level configuration -- CubeMars AK-series V1.x (SocketCAN)
# ---------------------------------------------------------------------------
@dataclass
class CubeMarsAkV1BusConfig:
    """Configuration for CubeMarsAkV1Bench (CubeMars V1.x firmware over SocketCAN).

    V1.x uses CAN 2.0A standard frames (11-bit arbitration IDs) over the
    same SocketCAN interface as the V3.0 motor -- both can share can1
    simultaneously because CAN hardware distinguishes frame types.

    Attributes
    ----------
    channel : SocketCAN interface name, e.g. "can1".
    motors  : Mapping of human-readable name -> CubeMarsAkV1MotorConfig.
    bitrate : CAN bitrate in bps (default 1 Mbps; do not change).
    """
    channel: str
    motors:  dict[str, CubeMarsAkV1MotorConfig] = field(default_factory=dict)
    bitrate: int = 1_000_000


# ---------------------------------------------------------------------------
# Default bench-test config (single DM-J4310-2EC on /dev/ttyACM0)
# ---------------------------------------------------------------------------
# This is the single source of truth for bench-test hardware wiring.
# _common.open_bus() / open_motor() read from here -- no separate globals.
# Edit here if your serial port, CAN ID, or Master ID differs.
DEFAULT_BENCH_CONFIG = DamiaoBusConfig(
    port="/dev/ttyACM0",
    baudrate=921600,
    motors={
        "j1": DamiaoMotorConfig(
            can_id=0x01,
            master_id=0x11,        # MUST match firmware MST_ID; factory default 0x00 is wrong
            model=DM_Motor_Type.DM4310,
        ),
    },
)


# ---------------------------------------------------------------------------
# Default bench-test config -- CubeMars AK60-6 V3.0 KV80 (elbow joint)
# ---------------------------------------------------------------------------
# Adapter  : DSDTech SH-C30A (USB ID 1d50:606f, gs_usb module -> can1)
# CAN ID   : 0x02 (confirmed; previous default 0x68 was R-Link factory value)
# CAN Mode : Query-Reply (motor replies when commanded)
# Firmware : AK60_6_SE_V3 (V3.0, extended 29-bit frames)
# Limits   : P=±12.5 rad, V=±45 rad/s, T=±15 N·m  (CUBEMARS_LIMITS["AK60-6"])
DEFAULT_AK60_6_BENCH_CONFIG = CubeMarsAkV3BusConfig(
    channel="can1",
    motors={"j1": CubeMarsMotorConfig(can_id=0x02, model="AK60-6")},
)


# ---------------------------------------------------------------------------
# Default bench-test config -- CubeMars AK80-8 KV60 V1.x (shoulder joint)
# ---------------------------------------------------------------------------
# Adapter  : DSDTech SH-C30A (same USB adapter as AK60-6, shares can1)
# CAN ID   : 0x01 (set via R-Link)
# Firmware : V1.x (standard 11-bit frames, position-first MIT layout)
# Limits   : P=±12.5 rad, V=±37.5 rad/s, T=±32 N·m  (CUBEMARS_LIMITS["AK80-8"])
DEFAULT_AK80_8_BENCH_CONFIG = CubeMarsAkV1BusConfig(
    channel="can1",
    motors={"j1": CubeMarsAkV1MotorConfig(can_id=0x01, model="AK80-8")},
)
