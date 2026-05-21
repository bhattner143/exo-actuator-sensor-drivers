"""ak_v1_bus.py -- SocketCAN wrapper for CubeMars AK-series V1.x motors.

High-level bus interface mirroring ``CubeMarsAkV3Bench`` / ``DamiaoBus`` so test
scripts look identical across all three motor families.

Hardware (this bench):
  Adapter : DSDTech SH-C30A USB-CAN (gs_usb, kernel module installed
            via install_gs_usb.sh -> can1 at 1 Mbps)
  Motor   : CubeMars AK80-8 V1.x KV60, ESC_ID = 0x01  (shoulder joint)
  Manual  : "AK Series Module Driver User Manual V1.0.15.X"

Protocol notes:
  V1.x firmware uses CAN 2.0A **standard frames** (11-bit IDs), unlike
  V3.0 which uses 29-bit extended frames.  Both motors can share can1
  simultaneously -- python-can distinguishes them via ``is_extended_id``.

  MIT mode must be activated by an explicit enter-mode frame (0xFF×7+0xFC)
  before the first MIT command.  ``connect()`` does this automatically.

  Each MIT command triggers exactly one feedback reply -- the motor is
  silent otherwise (Query-Reply behaviour).  Therefore ``write()`` sends
  the MIT command AND polls for one feedback frame so that ``read_state()``
  always returns fresh values.

Units convention:
  All setpoints and feedback are in SI units: radians, rad/s, amps (Iq).
  The ``read_raw()`` helper returns the same quantities plus temperature
  (°C) and the raw error flag byte for diagnostics.
"""
from __future__ import annotations

import time

import can
from .AK_V1_CAN import CubeMarsAkV1Motor
from motor_config import CubeMarsAkV1BusConfig, CubeMarsAkV1MotorConfig, DEFAULT_AK80_8_BENCH_CONFIG


class CubeMarsAkV1Bench:
    """Context-managed wrapper around one CubeMarsAkV1Motor on SocketCAN.

    Mirrors the high-level shape of ``CubeMarsAkV3Bench`` and ``DamiaoBus``:

        with CubeMarsAkV1Bench(config) as bus:
            bus.write("goal_position", {"j1": 0.5}, kp=40, kd=1.5)
            q, dq, ia = bus.read_state()["j1"]   # rad, rad/s, A

    NOTE: ``ia`` (third element of the state tuple) is **amps** (Iq),
    not N·m.  Multiply by the motor's Kt to get torque.

    ``write()`` always sends the MIT command and then polls for one
    feedback frame, so the motor state cached in the underlying
    ``CubeMarsAkV1Motor`` is fresh after every call.  ``read_state()`` and
    ``read_raw()`` return that cached state without sending any CAN frame.
    """

    def __init__(self, config: CubeMarsAkV1BusConfig, name: str = "j1") -> None:
        if name not in config.motors:
            raise KeyError(f"CubeMarsAkV1Bench: motor '{name}' not in config.motors")
        self.config: CubeMarsAkV1BusConfig    = config
        self.name:   str              = name
        self._motor_cfg: CubeMarsAkV1MotorConfig = config.motors[name]
        self._bus:  can.BusABC | None = None
        self._motor: CubeMarsAkV1Motor | None = None
        self._is_connected = False

    # ---- lifecycle -----------------------------------------------------

    def connect(self, *, set_zero: bool = False) -> "CubeMarsAkV1Bench":
        """Open the SocketCAN bus, put the motor into MIT mode.

        Args:
            set_zero: If True, latch the current shaft angle as 0 rad
                      (RAM-only, volatile -- lost on power cycle) before
                      returning.
        """
        self._bus = can.Bus(
            channel=self.config.channel,
            interface="socketcan",
        )
        self._motor = CubeMarsAkV1Motor(self._bus, can_id=self._motor_cfg.can_id)

        # Drain stale frames buffered since the last script run.
        self._drain(0.05)

        self._motor.enter_mode()
        time.sleep(0.05)

        if set_zero:
            self._motor.set_zero()
            time.sleep(0.05)

        # Initial zero-torque ping to confirm the link and populate state.
        self._motor.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        self._motor.poll_feedback(timeout=0.3)

        self._is_connected = True
        return self

    def disconnect(self) -> None:
        """Exit MIT mode (motor goes limp) and close the SocketCAN bus."""
        if self._motor is not None:
            try:
                self._motor.exit_mode()
                time.sleep(0.02)
            except Exception:
                pass
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        self._is_connected = False

    def __enter__(self) -> "CubeMarsAkV1Bench":
        if not self._is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    # ---- private helpers -----------------------------------------------

    def _drain(self, timeout: float = 0.05) -> None:
        """Discard frames currently queued on the bus (avoids stale reads)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._bus.recv(timeout=0.01) is None:
                return

    def _check_connected(self, caller: str) -> None:
        if not self._is_connected:
            raise RuntimeError(f"CubeMarsAkV1Bench.{caller}(): not connected")
        if self.name not in self.config.motors:
            raise KeyError(f"CubeMarsAkV1Bench: no motor '{self.name}' in config")

    # ---- I/O -----------------------------------------------------------

    def write(self, data_name: str, values: dict, *,
              kp: float = 0.0, kd: float = 0.0,
              dq_des: float = 0.0, tau_ff: float = 0.0) -> None:
        """Send one MIT frame, then poll for the motor's reply.

        The feedback reply updates the cached state accessible via
        ``read_state()`` and ``read_raw()``.

        Supported data_name strings (units = rad / rad/s / N·m):
          - ``"goal_position"``  MIT position hold; kp > 0, kd > 0.
                                  values[name] = q_des (rad).
                                  NEVER set kd = 0 in position mode.
          - ``"goal_velocity"``  MIT velocity track; kd > 0.
                                  values[name] = dq_des (rad/s).
          - ``"goal_torque"``    MIT open-loop torque; kp = kd = 0.
                                  values[name] = tau_ff (N·m).
          - ``"mit_command"``    Raw MIT pass-through.
                                  values[name] = q_des; + dq_des, kp, kd, tau_ff.
        """
        self._check_connected("write")
        if self.name not in values:
            raise KeyError(f"CubeMarsAkV1Bench: expected key {self.name!r} in values")
        v = float(values[self.name])
        m = self._motor

        if data_name == "goal_position":
            m.set_mit(v, dq_des, kp, kd, tau_ff)
        elif data_name == "goal_velocity":
            m.set_mit(0.0, v, 0.0, kd, tau_ff)
        elif data_name == "goal_torque":
            m.set_mit(0.0, 0.0, 0.0, 0.0, v)
        elif data_name == "mit_command":
            m.set_mit(v, dq_des, kp, kd, tau_ff)
        else:
            raise ValueError(f"CubeMarsAkV1Bench: unknown data_name {data_name!r}")

        # V1.x motor replies to every MIT command; poll to update cached state.
        m.poll_feedback(timeout=0.05)

    def read_state(self, timeout: float = 0.1) -> dict:
        """Return the latest decoded state in SI units.

        Returns ``{name: (q_rad, dq_rad_s, current_a)}``.

        State is populated by the most recent ``write()`` call.  No CAN
        frame is sent here -- call ``write()`` first each control tick.
        The ``timeout`` argument is accepted for API parity with
        ``CubeMarsAkV3Bench.read_state()`` but is not used.
        """
        self._check_connected("read_state")
        m = self._motor
        return {self.name: (m.pos_rad, m.vel_rad_s, m.current_a)}

    def read_raw(self, timeout: float = 0.1) -> dict:
        """Like ``read_state`` but includes temperature and error flag.

        Returns ``{name: (pos_rad, vel_rad_s, current_a, temp_c, error)}``.
        """
        self._check_connected("read_raw")
        m = self._motor
        return {self.name: (m.pos_rad, m.vel_rad_s, m.current_a,
                            m.temp_c, m.error)}

    def set_zero(self) -> None:
        """Latch current shaft angle as 0 rad (RAM only, volatile)."""
        self._check_connected("set_zero")
        self._motor.set_zero()

    def motor_names(self) -> list[str]:
        """Return list of registered motor names."""
        return [self.name]


# ---------------------------------------------------------------------------
# Convenience factory
# ---------------------------------------------------------------------------

def open_cubemars_ak_v1_bench(*, config: CubeMarsAkV1BusConfig | None = None,
                     set_zero: bool = False) -> CubeMarsAkV1Bench:
    """Convenience constructor using DEFAULT_AK80_8_BENCH_CONFIG.

    Hardware wiring is read from ``motor_config.DEFAULT_AK80_8_BENCH_CONFIG``
    (channel, CAN ID, model limits).  Edit that config to change hardware.

    Use as a context manager::

        from _common import open_cubemars_ak_v1_bench

        with open_cubemars_ak_v1_bench(set_zero=True) as bus:
            bus.write("goal_position", {"j1": 0.5}, kp=40, kd=1.5)
            q, dq, ia = bus.read_state()["j1"]   # rad, rad/s, A
    """
    cfg = config if config is not None else DEFAULT_AK80_8_BENCH_CONFIG
    bench = CubeMarsAkV1Bench(cfg)
    bench.connect(set_zero=set_zero)
    return bench
