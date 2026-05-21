"""ak_v3_bus.py -- SocketCAN wrapper for CubeMars AK-series V3.0 motors.

High-level bus interface mirroring DamiaoBus for parallel test script design.
Targets V3.0 firmware over SocketCAN (can1, 1 Mbps) using CAN 2.0B extended
frames with packet types encoded into the upper 21 bits of the arbitration ID.

Hardware (this bench):
  Adapter : DSDTech SH-C30A USB-CAN (gs_usb, kernel module installed
            via install_gs_usb.sh -> can1 at 1 Mbps)
  Motor   : CubeMars AK60-6 V3.0 KV80, ESC_ID = 0x02
  Manual  : "AK Series Module Product Manual V3.0.1"

Units convention:
  The motor's CAN protocol uses degrees, ERPM (electrical RPM), and amps.
  MIT commands take radians and rad/s; feedback is decoded to (deg, ERPM, A).
  For convenience this wrapper exposes BOTH unit systems via read_state()
  (rad / rad/s / A_proxy_for_torque) and read_raw() (deg / ERPM / A).
"""
from __future__ import annotations

import time
import math

import can
from .AK_V3_CAN import CubeMarsAkV3Motor
from .ak_v3_common import CUBEMARS_AK_V3_LIMITS
from motor_config import CubeMarsAkV3BusConfig, DEFAULT_AK60_6_BENCH_CONFIG


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


class CubeMarsAkV3Bench:
    """Context-managed wrapper around one CubeMarsAkV3Motor on SocketCAN.

    Mirrors the high-level shape of ``DamiaoBus`` so test scripts feel
    parallel between the two motor families:

        with CubeMarsAkV3Bench(channel="can1", can_id=0x02) as bus:
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
        self.motor: CubeMarsAkV3Motor | None = None
        self._is_connected = False
        self._pole_pairs = _AK_V3_POLE_PAIRS.get(model, 14)

    # ---- lifecycle -----------------------------------------------------
    def connect(self, *, enable: bool = False, set_zero: bool = False) -> "CubeMarsAkV3Bench":
        """Open the SocketCAN bus and bind one CubeMarsAkV3Motor.

        Args:
            enable:   V3.0 firmware has no separate enable frame -- this
                      arg is accepted for API parity with DamiaoBus and
                      simply triggers one harmless zero-torque MIT frame
                      so the motor's first feedback frame is fresh.
            set_zero: Latch current shaft angle as 0 (RAM only, erased
                      on power loss).  See CubeMarsAkV3Motor.set_origin().
        """
        self.bus = can.Bus(channel=self.channel, interface="socketcan")
        self.motor = CubeMarsAkV3Motor(self.bus, can_id=self.can_id, model=self.model)
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

    def __enter__(self) -> "CubeMarsAkV3Bench":
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        """True between successful connect() and disconnect()."""
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        """Internal flag to track connection state."""
        self._is_connected = value

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
            raise RuntimeError("CubeMarsAkV3Bench: not connected")
        if self.name not in values:
            raise KeyError(f"CubeMarsAkV3Bench: expected key {self.name!r} in values")
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
            raise ValueError(f"CubeMarsAkV3Bench: unknown data_name {data_name!r}")

    def read_state(self, timeout: float = 0.1) -> dict:
        """Block for one fresh feedback frame and return decoded SI units.

        Returns: ``{name: (q_rad, dq_rad_s, current_a)}``.
        Note ``current_a`` is Iq in amps -- multiply by motor Kt for Nm.
        """
        if not self.is_connected:
            raise RuntimeError("CubeMarsAkV3Bench: not connected")
        # Trigger a reply if firmware is in Query-Reply mode.
        self.motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        if not self.motor.poll_feedback(timeout=timeout):
            # Return cached values if no fresh frame arrived.
            pass
        q_rad     = math.radians(self.motor.pos_deg)
        # ERPM -> mechanical RPM -> rad/s
        mech_rpm  = self.motor.spd_erpm / self._pole_pairs
        dq_rad_s  = mech_rpm * 2.0 * math.pi / 60.0
        return {self.name: (q_rad, dq_rad_s, self.motor.current_a)}

    def read_raw(self, timeout: float = 0.1) -> dict:
        """Like ``read_state`` but in native units: (deg, ERPM, A, temp_C, err)."""
        if not self.is_connected:
            raise RuntimeError("CubeMarsAkV3Bench: not connected")
        self.motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        self.motor.poll_feedback(timeout=timeout)
        m = self.motor
        return {self.name: (m.pos_deg, m.spd_erpm, m.current_a, m.temp_c, m.error)}

    def set_zero(self, *, permanent: bool = False) -> None:
        """Latch the current shaft angle as 0.  ``permanent=True`` burns flash."""
        if self.motor is None:
            raise RuntimeError("CubeMarsAkV3Bench: not connected")
        self.motor.set_origin(permanent=permanent)

    def motor_names(self) -> list[str]:
        """Return list of registered motor names (just 'j1' for single motor)."""
        return [self.name]


def open_cubemars_ak_v3_bench(*, config: CubeMarsAkV3BusConfig | None = None,
                     set_zero: bool = False) -> CubeMarsAkV3Bench:
    """Convenience constructor using DEFAULT_AK60_6_BENCH_CONFIG.

    Hardware wiring is read from ``motor_config.DEFAULT_AK60_6_BENCH_CONFIG``
    (channel, CAN ID, model limits).  Edit that config to change hardware.

    Use as a context manager::

        with open_cubemars_ak_v3_bench(set_zero=False) as bus:
            bus.write("goal_position", {"j1": 0.0}, kp=20, kd=1.0)
            q, dq, i = bus.read_state()["j1"]
    """
    cfg = config if config is not None else DEFAULT_AK60_6_BENCH_CONFIG
    j1  = next(iter(cfg.motors.values()))
    bench = CubeMarsAkV3Bench(channel=cfg.channel, can_id=j1.can_id, model=j1.model)
    bench.connect(set_zero=set_zero)
    return bench
