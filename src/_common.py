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

# Add src/ (this file's own directory) to sys.path so DM_CAN.py is importable
# without any package installation.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from DM_CAN import (
    Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable,
)
from motor_config import (
    DamiaoBusConfig, DamiaoMotorConfig, DEFAULT_BENCH_CONFIG,
    CubeMarsBusConfig, CubeMarsMotorConfig, DEFAULT_CUBEMARS_BENCH_CONFIG,
)
from damiao_bus import DamiaoBus
from cubemars_bus import CubemarsMotorsBus


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


def open_cubemars_bus(*, enable=True, set_zero=False):
    """Open a CubemarsMotorsBus using DEFAULT_CUBEMARS_BENCH_CONFIG.

    CubeMars driver is MIT-only -- no mode switch needed (unlike DamiaoBus).
    Limits are taken from CUBEMARS_LIMITS, not read from firmware.

    Args:
        enable:   Send enable frame (FF...FC) to all motors on connect.
        set_zero: Latch the current shaft angle as 0 rad.

    Returns:
        Connected CubemarsMotorsBus. Use as a context manager:

            with open_cubemars_bus(set_zero=True) as bus:
                bus.write("goal_position", {"j1": 1.0}, kp=30, kd=1.0)
                q, dq, tau = bus.read_state()["j1"]
    """
    bus = CubemarsMotorsBus(DEFAULT_CUBEMARS_BENCH_CONFIG)
    bus.connect(enable=enable, set_zero=set_zero)
    return bus


# ---------------------------------------------------------------------------
# CubeMars AK60-6 V3.0 helpers -- SocketCAN via gs_usb (DSDTech SH-C30A)
# ---------------------------------------------------------------------------
# These helpers target V3.0 firmware over SocketCAN (can1, 1 Mbps).
# Unlike the legacy HDSC serial driver in cubemars_bus.py, V3.0 firmware
# uses CAN 2.0B extended frames with packet types encoded into the upper
# 21 bits of the arbitration ID -- see src/ak_v3_can.py.
#
# Hardware (this bench):
#   Adapter : DSDTech SH-C30A USB-CAN (gs_usb, kernel module installed
#             via install_gs_usb.sh -> can1 at 1 Mbps)
#   Motor   : CubeMars AK60-6 V3.0 KV80, ESC_ID = 0x02
#   Manual  : "AK Series Module Product Manual V3.0.1"
#
# Units convention:
#   The motor's CAN protocol uses degrees, ERPM (electrical RPM), and amps.
#   MIT commands take radians and rad/s; feedback is decoded to (deg, ERPM, A).
#   For convenience this wrapper exposes BOTH unit systems via read_state()
#   (rad / rad/s / A_proxy_for_torque) and read_raw() (deg / ERPM / A).
import can as _can  # imported lazily so non-CAN tests don't require python-can
import math as _math
from ak_v3_can import AkV3Motor as _AkV3Motor
from ak_v3_common import AK_V3_LIMITS as _AK_V3_LIMITS

# Pole-pair counts for ERPM -> mechanical-RPM conversion (manual table p.42).
# AK60-6 KV80 uses 14 pole pairs (verified via R-Link "poles 14x2" display).
_AK_V3_POLE_PAIRS = {
    "AK10-9":  21,
    "AK60-6":  14,
    "AK70-9":  14,
    "AK80-9":  21,
    "AKE60-8": 14,
    "AKE80-8": 21,
}


class AkV3Bench:
    """Context-managed wrapper around one AkV3Motor on SocketCAN.

    Mirrors the high-level shape of ``DamiaoBus`` so test scripts feel
    parallel between the two motor families:

        with AkV3Bench(channel="can1", can_id=0x02) as bus:
            bus.write("goal_position", {"j1": 1.57}, kp=30, kd=1.0)
            q, dq, tau = bus.read_state()["j1"]   # radians, rad/s, A

    NOTE: ``tau`` returned by ``read_state()`` is **amps** (Iq), not Nm.
    Convert to Nm with ``Iq * Kt`` if needed (Kt depends on the motor).
    """

    def __init__(self, channel: str = "can1",
                 can_id: int = 0x02,
                 model: str = "AK60-6",
                 bitrate: int = 1_000_000,
                 name: str = "j1") -> None:
        self.channel = channel
        self.can_id  = can_id
        self.model   = model
        self.bitrate = bitrate
        self.name    = name
        self.bus = None
        self.motor: _AkV3Motor | None = None
        self.is_connected = False
        self._pole_pairs = _AK_V3_POLE_PAIRS.get(model, 14)

    # ---- lifecycle -----------------------------------------------------
    def connect(self, *, enable: bool = False, set_zero: bool = False) -> "AkV3Bench":
        """Open the SocketCAN bus and bind one AkV3Motor.

        Args:
            enable:   V3.0 firmware has no separate enable frame -- this
                      arg is accepted for API parity with DamiaoBus and
                      simply triggers one harmless zero-torque MIT frame
                      so the motor's first feedback frame is fresh.
            set_zero: Latch current shaft angle as 0 (RAM only, erased
                      on power loss).  See AkV3Motor.set_origin().
        """
        self.bus = _can.Bus(channel=self.channel, interface="socketcan")
        self.motor = _AkV3Motor(self.bus, can_id=self.can_id, model=self.model)
        # Drain any stale feedback buffered by the kernel before this script ran.
        self._drain(0.05)
        if set_zero:
            self.motor.set_origin(permanent=False)
            time.sleep(0.05)
            self._drain(0.05)
        if enable:
            # Send a zero-MIT to fetch one fresh feedback frame.
            self.motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            self.motor.poll_feedback(timeout=0.2)
        self.is_connected = True
        return self

    def disconnect(self) -> None:
        """Stop the motor (zero-torque MIT) and close the bus."""
        if self.motor is not None:
            try:
                self.motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.is_connected = False

    def __enter__(self) -> "AkV3Bench":
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ---- I/O -----------------------------------------------------------
    def _drain(self, timeout: float = 0.05) -> None:
        """Discard any frames currently queued on the bus."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=0.01)
            if msg is None:
                return

    def write(self, data_name: str, values: dict, *,
              kp: float = 0.0, kd: float = 0.0,
              dq_des: float = 0.0, tau_ff: float = 0.0,
              acc_erpm_s2: int = 30_000) -> None:
        """Send one control frame.  ``values`` maps motor-name -> setpoint.

        Supported data_name (units = radians / rad/s / N·m proxy):
          - "goal_position"  MIT position hold;     kp>0, kd>0; values[name]=q_rad
          - "goal_velocity"  MIT velocity track;    kd>0;       values[name]=dq_rad_s
          - "goal_torque"    MIT open-loop torque;              values[name]=tau_Nm
          - "mit_command"    Raw MIT pass-through;              values[name]=q_rad
                              + dq_des, kp, kd, tau_ff
          - "goal_position_deg"  Servo Position Loop (manual §4.1.5)
                                 values[name] = degrees in [-36000, 36000]
          - "goal_pos_spd"   Servo Position-Velocity Loop (manual §4.1.7)
                              values[name] = degrees; dq_des in ERPM; acc_erpm_s2
          - "goal_speed_erpm"  Servo Velocity Loop (manual §4.1.4)
                                values[name] = ERPM
          - "goal_duty"      Servo Duty Cycle Mode (manual §4.1.1)
                              values[name] in [-1, 1]
        """
        if not self.is_connected:
            raise RuntimeError("AkV3Bench: not connected")
        if self.name not in values:
            raise KeyError(f"AkV3Bench: expected key {self.name!r} in values")
        v = float(values[self.name])
        m = self.motor

        if data_name == "goal_position":
            m.set_mit(p_des=v, v_des=dq_des, kp=kp, kd=kd, t_ff=tau_ff)
        elif data_name == "goal_velocity":
            m.set_mit(p_des=0.0, v_des=v, kp=0.0, kd=kd, t_ff=tau_ff)
        elif data_name == "goal_torque":
            m.set_mit(p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=v)
        elif data_name == "mit_command":
            m.set_mit(p_des=v, v_des=dq_des, kp=kp, kd=kd, t_ff=tau_ff)
        elif data_name == "goal_position_deg":
            m.set_position_deg(v)
        elif data_name == "goal_pos_spd":
            m.set_pos_spd(v, int(dq_des), int(acc_erpm_s2))
        elif data_name == "goal_speed_erpm":
            m.set_rpm(v)
        elif data_name == "goal_duty":
            m.set_duty(v)
        else:
            raise ValueError(f"AkV3Bench: unknown data_name {data_name!r}")

    def read_state(self, timeout: float = 0.1) -> dict:
        """Block for one fresh feedback frame and return decoded SI units.

        Returns: ``{name: (q_rad, dq_rad_s, current_a)}``.
        Note ``current_a`` is Iq in amps -- multiply by motor Kt for Nm.
        """
        if not self.is_connected:
            raise RuntimeError("AkV3Bench: not connected")
        # Trigger a reply if firmware is in Query-Reply mode.
        self.motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        if not self.motor.poll_feedback(timeout=timeout):
            # Return cached values if no fresh frame arrived.
            pass
        q_rad     = _math.radians(self.motor.pos_deg)
        # ERPM -> mechanical RPM -> rad/s
        mech_rpm  = self.motor.spd_erpm / self._pole_pairs
        dq_rad_s  = mech_rpm * 2.0 * _math.pi / 60.0
        return {self.name: (q_rad, dq_rad_s, self.motor.current_a)}

    def read_raw(self, timeout: float = 0.1) -> dict:
        """Like ``read_state`` but in native units: (deg, ERPM, A, temp_C, err)."""
        if not self.is_connected:
            raise RuntimeError("AkV3Bench: not connected")
        self.motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        self.motor.poll_feedback(timeout=timeout)
        m = self.motor
        return {self.name: (m.pos_deg, m.spd_erpm, m.current_a, m.temp_c, m.error)}

    def set_zero(self, *, permanent: bool = False) -> None:
        """Latch the current shaft angle as 0.  ``permanent=True`` burns flash."""
        if self.motor is None:
            raise RuntimeError("AkV3Bench: not connected")
        self.motor.set_origin(permanent=permanent)


def open_ak_v3_bench(*, channel: str = "can1", can_id: int = 0x02,
                     model: str = "AK60-6", set_zero: bool = False) -> AkV3Bench:
    """Convenience constructor for the V3.0 bench (DSDTech SH-C30A on can1).

    Use as a context manager:

        with open_ak_v3_bench(set_zero=False) as bus:
            bus.write("goal_position", {"j1": 0.0}, kp=20, kd=1.0)
            q, dq, i = bus.read_state()["j1"]
    """
    bench = AkV3Bench(channel=channel, can_id=can_id, model=model)
    bench.connect(set_zero=set_zero)
    return bench
