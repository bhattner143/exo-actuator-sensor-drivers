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

Quick-start example (2 Damiao + 2 CubeMars)
--------------------------------------------
    from motor_config import (
        DamiaoBusConfig, DamiaoMotorConfig,
        CubeMarsBusConfig, CubeMarsMotorConfig,
    )

    damiao_cfg = DamiaoBusConfig(
        port="/dev/ttyACM0",
        motors={
            "shoulder": DamiaoMotorConfig(can_id=0x01),
            "elbow":    DamiaoMotorConfig(can_id=0x02),
        },
    )
    cubemars_cfg = CubeMarsBusConfig(
        port="/dev/ttyACM1",
        motors={
            "wrist":   CubeMarsMotorConfig(can_id=0x01, model="AK80-9"),
            "gripper": CubeMarsMotorConfig(can_id=0x02, model="AK60-6"),
        },
    )
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

# Ensure DM_CAN.py is importable from this same src/ directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from DM_CAN import DM_Motor_Type

# ---------------------------------------------------------------------------
# CubeMars AK-series MIT limits  [PMAX (rad), VMAX (rad/s), TMAX (N·m)]
# ---------------------------------------------------------------------------
# Source: CubeMars datasheets + MIT-controller community tables.
# These are the values the firmware uses for bit-packing -- they define the
# full-scale range of the 16/12-bit position/velocity/torque fields.
CUBEMARS_LIMITS: dict[str, list[float]] = {
    "AK10-9":  [12.5,  50.0,  65.0],   # high-torque, low-speed planetary
    "AK60-6":  [12.5,  45.0,  15.0],   # medium-duty
    "AK70-10": [12.5,  50.0,  24.8],   # medium-duty, higher speed
    "AK80-6":  [12.5,  45.0,  15.0],   # same limits as AK60-6
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
# Bus-level configuration -- CubeMars
# ---------------------------------------------------------------------------
@dataclass
class CubeMarsBusConfig:
    """Configuration for the CubemarsMotorsBus (one HDSC USB-to-CAN serial port).

    Attributes
    ----------
    port     : Serial device path, e.g. "/dev/ttyACM1".
    motors   : Mapping of human-readable name -> CubeMarsMotorConfig.
    baudrate : Serial baud rate.  921600 for HDSC adapter.
    """
    port:     str
    motors:   dict[str, CubeMarsMotorConfig] = field(default_factory=dict)
    baudrate: int = 921600


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
# Default bench-test config (single CubeMars AK60-6 V3.0 KV80)
# ---------------------------------------------------------------------------
# Uses the SAME HDSC USB-to-CAN adapter as the Damiao motor -- swap the
# CAN H/L leads between motors; they are not run concurrently on this bench.
#
# CAN ID confirmed via 00_scan_can_id.py: 104 decimal (0x68).
# CAN Mode: set to "Inquiry Feedback" (MIT mode) in R-Link.
#   "Periodic Feedback" (Servo mode) will NOT respond to this driver.
# Model "AK60-6" limits (confirmed from R-Link: Hw=AK60_6V, Sw=AK60_6_SE_V3):
#   P_MAX = 12.5 rad, V_MAX = 45 rad/s, T_MAX = 15 N.m
DEFAULT_CUBEMARS_BENCH_CONFIG = CubeMarsBusConfig(
    port="/dev/ttyACM0",
    baudrate=921600,
    motors={
        "j1": CubeMarsMotorConfig(can_id=0x68, model="AK60-6"),
    },
)
