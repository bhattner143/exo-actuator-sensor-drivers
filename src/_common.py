"""_common.py -- Shared setup for all Damiao motor test scripts (tests/01-10).

This module centralises three concerns so each individual test script stays
short and focused on the control logic rather than boilerplate:

  1. Hardware wiring   -- imported from motor_config.DEFAULT_BENCH_CONFIG
                          (single source of truth; edit motor_config.py to
                          change port, CAN ID, or Master ID)
  2. Low-level helpers -- open_motor() for raw DM_CAN access (tests 01-02)
  3. Bus helpers       -- open_bus() returning a DamiaoBus for tests 03-10

Physical setup assumed by DEFAULT_BENCH_CONFIG:
  Motor  : Damiao DM-J4310-2EC V1.1 (CAN-ID = 0x01, Master-ID = 0x11)
  Adapter: HDSC USB-to-CAN (USB ID 2e88:4603), appears as /dev/ttyACM0
  Bus    : CAN 1 Mbps standard frame

Firmware parameters confirmed via Damiao debug tool:
    PMAX = 12.5 rad,  VMAX = 30 rad/s,  TMAX = 10 N.m,  GR = 10

IMPORTANT -- unit convention:
    The DM-J4310-2EC has dual 14-bit magnetic encoders; the output-shaft
    encoder is the source of truth for the firmware.  ALL CAN commanded
    values (p_des, v_des, tau_ff, q in MIT) and ALL feedback values
    (getPosition, getVelocity, getTorque) are in OUTPUT-SHAFT units.
    The 10:1 gearbox is handled internally by the firmware.
    Do NOT multiply any commanded or read value by the gear ratio.

IMPORTANT -- Master_ID:
    The motor's firmware parameter MST_ID must be 0x11 (not the factory
    default 0x00).  With 0x00 the Python driver's feedback filter drops
    all replies -> getPosition() always returns 0 -> motor races to the
    wrong position.  Set MST_ID = 0x11 via the Damiao Windows debug tool
    (Write Param tab) before running any test script.
"""
import sys
import os
import time
import serial

# Add src/ (this file's own directory) to sys.path so package imports work.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from damiao.DM_CAN import (
    Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable,
)
from motor_config import (
    DamiaoBusConfig, DamiaoMotorConfig, DEFAULT_BENCH_CONFIG,
    DEFAULT_AK60_6_BENCH_CONFIG, DEFAULT_AK80_8_BENCH_CONFIG,
)
from damiao.damiao_bus import DamiaoBus
from cubemars.ak_v3.ak_v3_bus import CubeMarsAkV3Bench, open_cubemars_ak_v3_bench
from cubemars.ak_v1.ak_v1_bus import CubeMarsAkV1Bench, open_cubemars_ak_v1_bench


def open_motor(motor_type=None, sync_limits=True):
    """Open the serial port and initialise the motor driver (raw DM_CAN API).

    Hardware wiring is read from motor_config.DEFAULT_BENCH_CONFIG so there
    is a single place to change port / CAN ID / Master ID.

    Args:
        motor_type: DM_Motor_Type enum member.  Defaults to the model in
                    DEFAULT_BENCH_CONFIG (DM4310 for J4310-2EC).
        sync_limits: If True (default), query PMAX/VMAX/TMAX from firmware.
                     Set False when doing read-only param dumps (01_read_params)
                     to avoid a double-read overhead.

    Returns:
        (motor, mc, ser) -- Motor, MotorControl, serial.Serial.
        Always pass all three to safe_disable_close() in a finally block.
    """
    j1_cfg = DEFAULT_BENCH_CONFIG.motors["j1"]
    if motor_type is None:
        motor_type = j1_cfg.model

    ser   = serial.Serial(DEFAULT_BENCH_CONFIG.port, DEFAULT_BENCH_CONFIG.baudrate,
                          timeout=0.5)
    motor = Motor(motor_type, j1_cfg.can_id, j1_cfg.master_id)
    mc    = MotorControl(ser)
    mc.addMotor(motor)

    if sync_limits:
        # Read the motor's actual PMAX/VMAX/TMAX from firmware.
        # This guards against firmware being set to a different range than
        # the library defaults -- a common source of position scaling errors.
        try:
            pmax = mc.read_motor_param(motor, DM_variable.PMAX)
            vmax = mc.read_motor_param(motor, DM_variable.VMAX)
            tmax = mc.read_motor_param(motor, DM_variable.TMAX)
            if None not in (pmax, vmax, tmax):
                mc.Limit_Param[int(motor_type)] = [pmax, vmax, tmax]
                print(f"[setup] Synced limits PMAX={pmax} VMAX={vmax} TMAX={tmax}")
            else:
                print("[setup] WARNING: could not read limits, using defaults")
        except Exception as e:
            print(f"[setup] WARNING: limit read failed: {e}")

    return motor, mc, ser


def safe_disable_close(motor, mc, ser):
    """Gracefully disable the motor and close the serial port.

    Always call this in a finally block so the motor is disabled even if
    the script is interrupted by Ctrl+C or an exception:

        motor, mc, ser = open_motor()
        try:
            ...  # control code
        finally:
            safe_disable_close(motor, mc, ser)

    Errors during disable/close are silently ignored so this function
    never raises -- a finaliser must not shadow the original exception.
    """
    try:
        mc.disable(motor)   # send [FF FF FF FF FF FF FF FD] to ESC_ID
    except Exception:
        pass
    try:
        ser.close()
    except Exception:
        pass


def open_bus(motor_type=None, *,
             mode=Control_Type.MIT,
             sync_limits=True, enable=True, set_zero=False):
    """Open a DamiaoBus using DEFAULT_BENCH_CONFIG (single motor ``"j1"``).

    Hardware wiring comes entirely from motor_config.DEFAULT_BENCH_CONFIG --
    no separate constants here.  Passing ``motor_type`` overrides only the
    model field of the j1 config (useful for testing a different variant).

    Args:
        motor_type:  DM_Motor_Type override (default: model in config).
        mode:        Firmware mode to switch into before enabling.
                     Control_Type.MIT / POS_VEL / VEL / TORQUE.
        sync_limits: Read PMAX/VMAX/TMAX from firmware on connect.
        enable:      Send the enable frame after switching mode.
        set_zero:    Latch the current shaft angle as 0 rad.

    Returns:
        Connected DamiaoBus.  Use as a context manager:

            with open_bus(mode=Control_Type.POS_VEL) as bus:
                bus.write("goal_pos_vel", {"j1": 1.57}, dq_des=5.0)
    """
    if motor_type is not None:
        import dataclasses
        j1_new = dataclasses.replace(DEFAULT_BENCH_CONFIG.motors["j1"],
                                     model=motor_type)
        cfg = dataclasses.replace(DEFAULT_BENCH_CONFIG,
                                  motors={"j1": j1_new})
    else:
        cfg = DEFAULT_BENCH_CONFIG
    bus = DamiaoBus(cfg)
    bus.connect(mode=mode, sync_limits=sync_limits,
                enable=enable, set_zero=set_zero)
    return bus
