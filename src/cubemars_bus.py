"""cubemars_bus.py -- LeRobot-style bus wrapper for CubeMars AK-series motors.

CubeMars AK motors use the same MIT bit-packed CAN frame as Damiao motors,
with two protocol differences:
  1. Feedback frame CAN ID = motor's own ESC_ID (not a separate Master ID).
  2. No firmware parameter read/write commands -- limits come from CUBEMARS_LIMITS.

This module implements the same ``read()`` / ``write()`` interface as
``DamiaoBus`` so the two buses can be used interchangeably in a control loop.

Architecture
------------
    CubeMarsBusConfig              -- declarative config (serialisable)
        └─ CubeMarsMotorConfig     -- per-motor: can_id, model string

    CubemarsMotorsBus              -- this class (self-contained serial driver)
        ├─ _ser : serial.Serial    -- raw serial port to HDSC USB-CAN adapter
        ├─ _limits : dict          -- name -> [PMAX, VMAX, TMAX]
        └─ _state  : dict          -- name -> (q, dq, tau) (cached feedback)

MIT frame encoding (identical to Damiao)
----------------------------------------
    Byte 0 : q[15:8]                -- position MSB
    Byte 1 : q[7:0]                 -- position LSB
    Byte 2 : dq[11:4]               -- velocity upper 8
    Byte 3 : dq[3:0] | Kp[11:8]    -- velocity lower 4 + Kp upper 4
    Byte 4 : Kp[7:0]                -- Kp lower 8
    Byte 5 : Kd[11:4]               -- Kd upper 8
    Byte 6 : Kd[3:0] | tau[11:8]   -- Kd lower 4 + torque upper 4
    Byte 7 : tau[7:0]               -- torque lower 8

Feedback frame (8-byte CAN payload, same layout as Damiao)
------------------------------------------------------------
    D[0]   : motor ESC_ID
    D[1:2] : POS[15:8], POS[7:0]    -> position (output shaft, rad)
    D[3]   : VEL[11:4]
    D[4]   : VEL[3:0] | TAU[11:8]
    D[5]   : TAU[7:0]               -> torque (N·m)
    D[6]   : T_MOS  (driver temperature, °C, not decoded here)
    D[7]   : T_ROTOR (motor temperature, °C, not decoded here)

Usage
-----
    from motor_config import CubeMarsBusConfig, CubeMarsMotorConfig
    from cubemars_bus import CubemarsMotorsBus

    cfg = CubeMarsBusConfig(
        port="/dev/ttyACM1",
        motors={
            "wrist":   CubeMarsMotorConfig(can_id=0x01, model="AK80-9"),
            "gripper": CubeMarsMotorConfig(can_id=0x02, model="AK60-6"),
        },
    )
    bus = CubemarsMotorsBus(cfg)
    bus.connect()
    pos = bus.read("position")
    bus.write("goal_position", {"wrist": 0.5}, kp=20.0, kd=0.8)
    bus.disconnect()
"""
from __future__ import annotations

import os
import sys
import time
import struct
import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from DM_CAN import float_to_uint, uint_to_float   # reuse the math helpers
from motor_config import CubeMarsBusConfig, CubeMarsMotorConfig, CUBEMARS_LIMITS

_READ_NAMES  = {"position", "velocity", "torque"}
_WRITE_NAMES = {"goal_position", "goal_velocity", "goal_torque", "mit_command"}

# HDSC USB-to-CAN 30-byte transmit envelope (identical to DM_CAN template).
# Bytes [13:15] carry the CAN ID; bytes [21:29] carry the 8-byte payload.
_SEND_FRAME = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)

# Length of one HDSC feedback frame from the adapter (bytes)
_FRAME_LEN = 16    # [0xAA, CMD, DLC, CAN_ID×4, payload×8, 0x55]

# Enable / disable / set-zero magic payloads (8 bytes)
_CMD_ENABLE  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
_CMD_DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
_CMD_ZERO    = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])


class CubemarsMotorsBus:
    """LeRobot-style bus for one or more CubeMars AK-series motors.

    Implements the same interface as DamiaoBus (read/write/connect/disconnect)
    so both buses can be driven identically in a control loop.

    All values are in output-shaft units (rad, rad/s, N·m).

    Internally manages:
      - Serial framing (HDSC USB-CAN 30-byte envelope)
      - MIT bit-packing using per-model PMAX/VMAX/TMAX from CUBEMARS_LIMITS
      - Feedback parsing and per-motor state cache
    """

    def __init__(self, config: CubeMarsBusConfig) -> None:
        self.config = config
        self._ser: serial.Serial | None = None
        self._buf: bytes = b""   # leftover bytes between recv calls

        # name -> [PMAX, VMAX, TMAX] -- looked up from CUBEMARS_LIMITS at init
        self._limits: dict[str, list[float]] = {
            name: CUBEMARS_LIMITS[cfg.model]
            for name, cfg in config.motors.items()
        }
        # name -> (q, dq, tau) -- last decoded feedback values
        self._state: dict[str, tuple[float, float, float]] = {
            name: (0.0, 0.0, 0.0) for name in config.motors
        }
        # can_id -> motor name -- reverse lookup for feedback routing
        self._id_to_name: dict[int, str] = {
            cfg.can_id: name for name, cfg in config.motors.items()
        }

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, *,
                enable: bool = True,
                set_zero: bool = False) -> None:
        """Open the serial port and optionally enable all motors.

        Note: CubeMars motors have no firmware parameter read API
        compatible with Damiao, so sync_limits is not supported.
        Limits are taken from CUBEMARS_LIMITS at construction time.

        Args:
            enable:   Send enable frame to all motors after opening port.
            set_zero: Latch current shaft angle as 0 rad. Default False.
        """
        self._ser = serial.Serial(
            self.config.port, self.config.baudrate, timeout=0.1
        )
        print(f"[CubemarsMotorsBus] opened {self.config.port}")

        if enable:
            self.enable_torque()

        if set_zero:
            self.set_zero()
            print("[CubemarsMotorsBus] zero positions latched")

    def disconnect(self) -> None:
        """Disable all motors and close the serial port."""
        if self._ser is not None and self._ser.is_open:
            self.disable_torque()
            self._ser.close()
        self._ser = None

    @property
    def is_connected(self) -> bool:
        """True between successful connect() and disconnect()."""
        return self._ser is not None and self._ser.is_open

    def __enter__(self) -> "CubemarsMotorsBus":
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    def enable_torque(self, names: list[str] | None = None) -> None:
        """Send the enable frame (FF FF FF FF FF FF FF FC) to motors."""
        targets = names if names is not None else list(self.config.motors)
        for name in targets:
            cfg = self.config.motors[name]
            self._send_raw(cfg.can_id, _CMD_ENABLE)
            time.sleep(0.05)
        self._recv_all()
        for name in targets:
            cfg = self.config.motors[name]
            print(
                f"[CubemarsMotorsBus] {name} "
                f"(CAN 0x{cfg.can_id:02X}, {cfg.model}) torque enabled"
            )
        time.sleep(0.1)

    def disable_torque(self, names: list[str] | None = None) -> None:
        """Send the disable frame (FF FF FF FF FF FF FF FD) to motors."""
        targets = names if names is not None else list(self.config.motors)
        for name in targets:
            cfg = self.config.motors[name]
            try:
                self._send_raw(cfg.can_id, _CMD_DISABLE)
                time.sleep(0.02)
                print(f"[CubemarsMotorsBus] {name} torque disabled")
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read(self, data_name: str,
             names: list[str] | None = None) -> dict[str, float]:
        """Read one physical quantity from one or more motors.

        Sends a minimal MIT frame (kp=kd=tau=0, q_des=current_q) to each
        motor to elicit a fresh feedback reply, then decodes and returns
        the requested field.

        Note: The status-poll command has kp=kd=0, so it applies zero
        torque and does not disturb the motor state.  If a write() is being
        called in the same loop iteration, call write() first to avoid the
        brief zero-torque window.

        Args:
            data_name: "position", "velocity", or "torque".
            names:     Motor names to query.  Default: all motors.

        Returns:
            dict of {name: value} in output-shaft units.
        """
        if data_name not in _READ_NAMES:
            raise ValueError(
                f"Unknown read name '{data_name}'. Choose from {_READ_NAMES}"
            )
        targets = names if names is not None else list(self.config.motors)
        result: dict[str, float] = {}

        for name in targets:
            cfg = self._resolve_cfg(name)
            lim = self._limits[name]
            q_cached = self._state[name][0]   # re-send toward current angle

            # Poll: send a null MIT frame (Kp=0, Kd=0, q_des=current) to
            # elicit a fresh feedback packet without disturbing the motor.
            payload = self._encode_mit(
                q=q_cached, dq=0.0, kp=0.0, kd=0.0, tau=0.0, lim=lim
            )
            self._send_raw(cfg.can_id, payload)
            time.sleep(0.002)   # ~2 ms for CAN round-trip at 1 Mbps
            self._recv_all()

            q, dq, tau = self._state[name]
            if data_name == "position":
                result[name] = q
            elif data_name == "velocity":
                result[name] = dq
            else:   # "torque"
                result[name] = tau

        return result

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write(self, data_name: str,
              values: dict[str, float],
              kp: float = 20.0,
              kd: float = 0.8,
              dq_des: float = 0.0,
              tau_ff: float = 0.0) -> None:
        """Send a command to one or more motors.

        Same semantics as DamiasBus.write():

            "goal_position"  -- MIT position hold:  controlMIT(kp, kd, q_des, 0, 0)
            "goal_velocity"  -- MIT velocity:        controlMIT(0, kd, 0, v_des, 0)
            "goal_torque"    -- open-loop torque:    controlMIT(0, 0, 0, 0, tau)
            "mit_command"    -- raw: value=q_des, remaining params from kwargs

        Args:
            data_name: See above.
            values:    {motor_name: setpoint} in rad | rad/s | N·m.
            kp:        MIT stiffness [0, 500].  Default 20 for AK-series.
            kd:        MIT damping   [0, 5].    NEVER 0 for position mode.
            dq_des:    Feed-forward velocity (rad/s).
            tau_ff:    Feed-forward torque (N·m).
        """
        if data_name not in _WRITE_NAMES:
            raise ValueError(
                f"Unknown write name '{data_name}'. Choose from {_WRITE_NAMES}"
            )
        for name, value in values.items():
            cfg = self._resolve_cfg(name)
            lim = self._limits[name]

            if data_name == "goal_position":
                payload = self._encode_mit(value, dq_des, kp, kd, tau_ff, lim)
            elif data_name == "goal_velocity":
                # Kp=0: position spring is absent; only velocity damping acts
                payload = self._encode_mit(0.0, value, 0.0, kd, tau_ff, lim)
            elif data_name == "goal_torque":
                # Kp=0, Kd=0: pure open-loop torque injection
                payload = self._encode_mit(0.0, 0.0, 0.0, 0.0, value, lim)
            else:   # "mit_command"
                payload = self._encode_mit(value, dq_des, kp, kd, tau_ff, lim)

            self._send_raw(cfg.can_id, payload)

        # Drain one round of feedback after writing all motors
        time.sleep(0.001)
        self._recv_all()

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def motor_names(self) -> list[str]:
        return list(self.config.motors)

    def get_state(self, name: str) -> tuple[float, float, float]:
        """Return cached (position, velocity, torque) for a motor."""
        if name not in self._state:
            raise KeyError(
                f"Motor '{name}' not found. Available: {list(self._state)}"
            )
        return self._state[name]

    def set_zero(self, names: list[str] | None = None) -> None:
        """Latch the current shaft angle of one or more motors as 0 rad."""
        targets = names if names is not None else list(self.config.motors)
        for name in targets:
            cfg = self.config.motors[name]
            self._send_raw(cfg.can_id, _CMD_ZERO)
            time.sleep(0.05)
        self._recv_all()

    def read_state(self, names: list[str] | None = None
                   ) -> dict[str, tuple[float, float, float]]:
        """Read (position, velocity, torque) in one CAN round-trip per motor.

        Roughly 3x faster than calling read() three times per loop tick.
        """
        targets = names if names is not None else list(self.config.motors)
        for name in targets:
            cfg = self._resolve_cfg(name)
            lim = self._limits[name]
            q_cached = self._state[name][0]
            payload = self._encode_mit(q_cached, 0.0, 0.0, 0.0, 0.0, lim)
            self._send_raw(cfg.can_id, payload)
        time.sleep(0.002)
        self._recv_all()
        return {name: self._state[name] for name in targets}

    # ------------------------------------------------------------------
    # MIT frame encode / decode
    # ------------------------------------------------------------------

    @staticmethod
    def _encode_mit(q: float, dq: float, kp: float, kd: float, tau: float,
                    lim: list[float]) -> bytes:
        """Pack 5 scalars into an 8-byte MIT CAN payload.

        Bit layout (identical to Damiao):
            Byte 0 : q[15:8]
            Byte 1 : q[7:0]
            Byte 2 : dq[11:4]
            Byte 3 : dq[3:0] | Kp[11:8]
            Byte 4 : Kp[7:0]
            Byte 5 : Kd[11:4]
            Byte 6 : Kd[3:0] | tau[11:8]
            Byte 7 : tau[7:0]

        Args:
            q, dq, kp, kd, tau : physical setpoints
            lim : [PMAX, VMAX, TMAX] for this motor model
        """
        pmax, vmax, tmax = lim[0], lim[1], lim[2]

        q_u   = float_to_uint(q,   -pmax,  pmax,  16)  # 16-bit position
        dq_u  = float_to_uint(dq,  -vmax,  vmax,  12)  # 12-bit velocity
        kp_u  = float_to_uint(kp,   0.0,   500.0, 12)  # 12-bit Kp
        kd_u  = float_to_uint(kd,   0.0,   5.0,   12)  # 12-bit Kd
        tau_u = float_to_uint(tau, -tmax,  tmax,  12)  # 12-bit torque FF

        data = bytearray(8)
        data[0] = (q_u >> 8) & 0xFF
        data[1] =  q_u       & 0xFF
        data[2] =  dq_u >> 4
        data[3] = ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF)
        data[4] =  kp_u & 0xFF
        data[5] =  kd_u >> 4
        data[6] = ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF)
        data[7] =  tau_u & 0xFF
        return bytes(data)

    @staticmethod
    def _decode_feedback(data: bytes | bytearray,
                         lim: list[float]) -> tuple[float, float, float]:
        """Decode an 8-byte MIT feedback payload into (q, dq, tau).

        Feedback layout (same as Damiao):
            D[0]   : motor ID (ignored here -- CAN ID used for routing)
            D[1:3] : POS (16-bit big-endian)
            D[3]   : VEL upper 8 bits
            D[4]   : VEL lower 4 | TAU upper 4
            D[5]   : TAU lower 8 bits
            D[6]   : T_MOS  (ignored)
            D[7]   : T_ROTOR (ignored)
        """
        pmax, vmax, tmax = lim[0], lim[1], lim[2]

        pos_u = np.uint16((data[1] << 8) | data[2])
        vel_u = np.uint16((data[3] << 4) | (data[4] >> 4))
        tau_u = np.uint16(((data[4] & 0xF) << 8) | data[5])

        q   = uint_to_float(pos_u, -pmax,  pmax,  16)
        dq  = uint_to_float(vel_u, -vmax,  vmax,  12)
        tau = uint_to_float(tau_u, -tmax,  tmax,  12)
        return q, dq, tau

    # ------------------------------------------------------------------
    # Serial I/O helpers
    # ------------------------------------------------------------------

    def _send_raw(self, can_id: int, payload: bytes) -> None:
        """Write one HDSC 30-byte envelope carrying the given 8-byte payload."""
        frame = _SEND_FRAME.copy()
        frame[13] =  can_id & 0xFF          # CAN ID bits [7:0]
        frame[14] = (can_id >> 8) & 0xFF    # CAN ID bits [10:8]
        frame[21:29] = np.frombuffer(payload, dtype=np.uint8)[:8]
        self._ser.write(bytes(frame))

    def _recv_all(self) -> None:
        """Read all available serial bytes, extract complete HDSC frames,
        decode feedback payloads, and update the per-motor state cache."""
        raw = self._buf + self._ser.read_all()
        frames, self._buf = self._extract_frames(raw)
        for frame in frames:
            cmd   = frame[1]
            if cmd != 0x11:
                continue    # not a motor feedback frame
            can_id = (frame[6] << 24) | (frame[5] << 16) | (frame[4] << 8) | frame[3]
            payload = frame[7:15]

            # For CubeMars, feedback CAN ID = ESC_ID directly.
            # Route to the matching motor by ESC_ID lookup.
            if can_id in self._id_to_name:
                name = self._id_to_name[can_id]
            elif can_id == 0 and len(payload) > 0:
                # Fallback: some firmware versions send CAN_ID=0 and embed
                # the motor ID in D[0] lower nibble.
                esc_id = payload[0] & 0x0F
                if esc_id in self._id_to_name:
                    name = self._id_to_name[esc_id]
                else:
                    continue
            else:
                continue    # feedback from unknown motor, ignore

            lim = self._limits[name]
            try:
                q, dq, tau = self._decode_feedback(payload, lim)
                self._state[name] = (q, dq, tau)
            except Exception:
                pass    # malformed packet -- skip

    @staticmethod
    def _extract_frames(data: bytes) -> tuple[list[bytes], bytes]:
        """Extract all complete 16-byte HDSC feedback frames from raw bytes.

        Returns (list_of_frames, leftover_bytes) where leftover_bytes is
        the partial frame (if any) at the end of the buffer.

        Frame structure: [0xAA, CMD, DLC, CAN_ID×4, payload×8, 0x55]
                          index  0    1    2   3..6     7..14   15
        Total = 16 bytes.
        """
        frames: list[bytes] = []
        i = 0
        last_complete = 0
        while i <= len(data) - _FRAME_LEN:
            if data[i] == 0xAA and data[i + _FRAME_LEN - 1] == 0x55:
                frames.append(data[i:i + _FRAME_LEN])
                i += _FRAME_LEN
                last_complete = i
            else:
                i += 1
        return frames, data[last_complete:]

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _resolve_cfg(self, name: str) -> CubeMarsMotorConfig:
        if name not in self.config.motors:
            raise KeyError(
                f"Motor '{name}' not found. Available: {list(self.config.motors)}"
            )
        if self._ser is None:
            raise RuntimeError(
                "CubemarsMotorsBus not connected. Call connect() first."
            )
        return self.config.motors[name]
