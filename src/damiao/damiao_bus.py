"""damiao_bus.py -- LeRobot-style bus wrapper for Damiao motors.

Wraps ``DM_CAN.Motor`` + ``DM_CAN.MotorControl`` behind a clean
``read()`` / ``write()`` interface that matches the LeRobot MotorsBus
contract.  Hardware wiring (serial port, CAN IDs, model) is fully
described by a ``DamiaoBusConfig`` dataclass -- no constants in control
code.

Architecture
------------
    DamiaoBusConfig            -- declarative config (serialisable)
        └─ DamiaoMotorConfig   -- per-motor: can_id, master_id, model

    DamiaoBus                  -- this class (wraps DM_CAN internals)
        ├─ _mc : MotorControl  -- vendor serial driver (DM_CAN.py)
        └─ _motors : dict      -- name -> Motor (state cache)

Supported data names
--------------------
    read()  : "position" (rad), "velocity" (rad/s), "torque" (N·m)
    write() : "goal_position", "goal_velocity", "goal_torque", "mit_command"

Usage
-----
    from motor_config import DamiasBusConfig, DamiaoMotorConfig
    from damiao_bus import DamiaoBus

    cfg = DamiaoBusConfig(
        port="/dev/ttyACM0",
        motors={
            "shoulder": DamiaoMotorConfig(can_id=0x01),
            "elbow":    DamiaoMotorConfig(can_id=0x02),
        },
    )
    bus = DamiaoBus(cfg)
    bus.connect()                  # open serial, sync limits, enable all

    pos = bus.read("position")     # {"shoulder": 0.52, "elbow": -0.17}
    bus.write("goal_position", {"shoulder": 1.57, "elbow": 0.0},
              kp=30.0, kd=1.0)

    bus.disconnect()               # disable all, close serial
"""
from __future__ import annotations

import os
import sys
import time
import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from .DM_CAN import Motor, MotorControl, Control_Type, DM_variable
from motor_config import DamiaoBusConfig, DamiaoMotorConfig

# Valid data-name strings for read() and write()
_READ_NAMES  = {"position", "velocity", "torque"}
# MIT-mode writes (CAN ID = ESC_ID, bit-packed frame):
#   goal_position : controlMIT(kp, kd, q, dq_des, tau_ff)
#   goal_velocity : controlMIT(0, kd, 0, v, tau_ff)
#   goal_torque   : controlMIT(0, 0, 0, 0, tau)
#   mit_command   : controlMIT(kp, kd, q, dq_des, tau_ff)  -- explicit
# Non-MIT writes (separate CAN IDs):
#   goal_pos_vel  : control_Pos_Vel(p_des, v_des)   CAN ID 0x100+ID
#   goal_speed    : control_Vel(v_des)              CAN ID 0x200+ID
_WRITE_NAMES = {
    "goal_position", "goal_velocity", "goal_torque", "mit_command",
    "goal_pos_vel", "goal_speed",
}


class DamiaoBus:
    """LeRobot-style bus for one or more Damiao motors on a single serial port.

    All commanded and feedback values are in **output-shaft units**
    (rad, rad/s, N·m).  The 10:1 gearbox is handled by the firmware
    transparently -- do not multiply by the gear ratio.

    Thread safety
    -------------
    Not thread-safe.  Call from a single control loop at 100 Hz or less.
    """

    def __init__(self, config: DamiaoBusConfig) -> None:
        self.config = config
        self._ser: serial.Serial | None = None
        self._mc:  MotorControl  | None = None
        # Human-readable name -> DM_CAN.Motor (populated in connect())
        self._motors: dict[str, Motor] = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, *,
                mode: Control_Type = Control_Type.MIT,
                sync_limits: bool = True,
                enable: bool = True,
                set_zero: bool = False) -> None:
        """Open serial port, register motors, switch mode, and (optionally) enable.

        Steps performed:
          1. Open serial port at the configured baud rate.
          2. Instantiate Motor + MotorControl, call addMotor() for each motor.
          3. If sync_limits=True, read PMAX/VMAX/TMAX from each motor's
             firmware via CAN and overwrite Limit_Param.  This ensures MIT
             bit-packing uses the correct full-scale range.  Strongly
             recommended -- skip only for very fast reconnections where you
             are certain the limits haven't changed.
          4. Switch every motor to ``mode`` (MIT / POS_VEL / VEL / TORQUE).
          5. If enable=True, send the enable frame to every motor.
          6. If set_zero=True, latch the current shaft angle as 0 rad.

        Args:
            mode:        Firmware control mode to switch into.
                         MIT for stiffness/damping, POS_VEL for set-and-forget
                         moves, VEL for constant-speed spin.
            sync_limits: Read firmware PMAX/VMAX/TMAX and sync Limit_Param.
            enable:      Send the enable frame to all motors.
            set_zero:    Latch current shaft angle as 0 rad on every motor.
        """
        self._ser = serial.Serial(self.config.port, self.config.baudrate, timeout=0.5)
        self._mc  = MotorControl(self._ser)

        # Register all motors; populate name -> Motor map
        for name, cfg in self.config.motors.items():
            m = Motor(cfg.model, cfg.can_id, cfg.master_id)
            self._mc.addMotor(m)
            self._motors[name] = m

        if sync_limits:
            self._sync_limits()

        # Switch every motor into the requested firmware mode *before* enabling.
        # Switching while enabled triggers an automatic disable on the firmware.
        for name, m in self._motors.items():
            self._mc.switchControlMode(m, mode)
        time.sleep(0.05)

        if enable:
            self.enable_torque()

        if set_zero:
            self.set_zero()
            print("[DamiaoBus] zero positions latched")

    def disconnect(self) -> None:
        """Disable all motors and close the serial port.

        Always call this in a ``finally`` block so the motor is disabled
        even if the control loop is interrupted by Ctrl+C or an exception.
        """
        if self._mc is not None:
            self.disable_torque()
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._mc = None
        self._ser = None
        self._motors.clear()

    @property
    def is_connected(self) -> bool:
        """True between successful connect() and disconnect()."""
        return self._ser is not None and self._ser.is_open

    def __enter__(self) -> "DamiaoBus":
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    def enable_torque(self, names: list[str] | None = None) -> None:
        """Send the enable frame to one or more motors.

        After enable_torque() the motor responds to control commands.
        Call this if you connected with ``enable=False`` (e.g. to switch
        mode after connect).
        """
        targets = names if names is not None else list(self._motors)
        for name in targets:
            m = self._resolve(name)
            self._mc.enable(m)
            print(f"[DamiaoBus] {name} (CAN 0x{m.SlaveID:02X}) torque enabled")
        time.sleep(0.2)

    def disable_torque(self, names: list[str] | None = None) -> None:
        """Send the disable frame to one or more motors.

        Safe to call multiple times; errors are swallowed so this is
        finalizer-safe.
        """
        targets = names if names is not None else list(self._motors)
        for name in targets:
            try:
                self._mc.disable(self._resolve(name))
                print(f"[DamiaoBus] {name} torque disabled")
            except Exception:
                pass   # best-effort; port may already be closed

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read(self, data_name: str,
             names: list[str] | None = None) -> dict[str, float]:
        """Read one physical quantity from one or more motors.

        Sends a CAN status-request frame to each requested motor and
        waits for the reply (one round-trip per motor, ~1 ms each on a
        1 Mbps bus).  Returns fresh values decoded from the firmware
        output-shaft encoder.

        Args:
            data_name: One of "position" (rad), "velocity" (rad/s),
                       "torque" (N·m).
            names:     List of motor names to query.  Defaults to all motors.

        Returns:
            dict mapping motor name -> value in output-shaft units.

        Example:
            >>> bus.read("position", ["shoulder", "elbow"])
            {"shoulder": 0.523, "elbow": -0.174}
        """
        if data_name not in _READ_NAMES:
            raise ValueError(
                f"Unknown read name '{data_name}'. Choose from {_READ_NAMES}"
            )
        targets = names if names is not None else list(self._motors)
        result: dict[str, float] = {}
        for name in targets:
            m = self._resolve(name)
            # refresh_motor_status sends a 0xCC status-request frame to the
            # motor and parses the reply into m.state_q / state_dq / state_tau
            self._mc.refresh_motor_status(m)
            if data_name == "position":
                result[name] = m.getPosition()
            elif data_name == "velocity":
                result[name] = m.getVelocity()
            else:   # "torque"
                result[name] = m.getTorque()
        return result

    def read_state(self, names: list[str] | None = None
                   ) -> dict[str, tuple[float, float, float]]:
        """Read (position, velocity, torque) from each motor in **one** CAN round-trip.

        Calling ``read("position")`` then ``read("velocity")`` then
        ``read("torque")`` sends three status-request frames per motor.
        ``read_state()`` sends one and decodes all three quantities, which
        roughly triples the achievable loop rate.

        Returns:
            dict mapping motor name -> ``(q, dq, tau)`` in output-shaft units.
        """
        targets = names if names is not None else list(self._motors)
        result: dict[str, tuple[float, float, float]] = {}
        for name in targets:
            m = self._resolve(name)
            self._mc.refresh_motor_status(m)
            result[name] = (m.getPosition(), m.getVelocity(), m.getTorque())
        return result

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write(self, data_name: str,
              values: dict[str, float],
              kp: float = 30.0,
              kd: float = 1.0,
              dq_des: float = 0.0,
              tau_ff: float = 0.0) -> None:
        """Send a command to one or more motors.

        Most modes use the MIT frame (CAN ID = ESC_ID).  Two non-MIT modes
        (goal_pos_vel, goal_speed) use the separate Position-Speed and
        Speed CAN IDs.  Call switch_mode() first to put the motor in the
        matching firmware mode (MIT, POS_VEL, or VEL).

            "goal_position"  -- MIT spring-damper position hold
                controlMIT(kp, kd, value, dq_des, tau_ff)
                kp > 0, kd > 0.  NEVER use kd=0 for position.

            "goal_velocity"  -- MIT velocity damping (Kp forced to 0)
                controlMIT(0, kd, 0, value, tau_ff)
                value = desired speed (rad/s).

            "goal_torque"    -- MIT open-loop torque (Kp=Kd=0)
                controlMIT(0, 0, 0, 0, value)
                value = desired torque (N·m).  Motor will accelerate
                if unloaded -- use with mechanical limits.

            "mit_command"    -- raw MIT: value is q_des, all 5 params
                controlMIT(kp, kd, value, dq_des, tau_ff)

            "goal_pos_vel"   -- Position-Speed mode (CAN ID 0x100+ID).
                control_Pos_Vel(value, dq_des)
                value = p_des (rad).  dq_des is reused as v_des cruise cap.
                Requires switch_mode(Control_Type.POS_VEL).

            "goal_speed"     -- Speed mode (CAN ID 0x200+ID).
                control_Vel(value).  value = v_des (rad/s).
                Requires switch_mode(Control_Type.VEL).

        Args:
            data_name: One of the four strings above.
            values:    {motor_name: setpoint} in rad | rad/s | N·m.
            kp:        MIT stiffness [0, 500].  Ignored for goal_velocity/torque.
            kd:        MIT damping   [0, 5].    Always used; NEVER 0 for position.
            dq_des:    Feed-forward velocity (rad/s).  Default 0.
            tau_ff:    Feed-forward torque (N·m).      Default 0.
        """
        if data_name not in _WRITE_NAMES:
            raise ValueError(
                f"Unknown write name '{data_name}'. Choose from {_WRITE_NAMES}"
            )
        for name, value in values.items():
            m = self._resolve(name)
            if data_name == "goal_position":
                # Spring-damper: tau = kp*(value - q) + kd*(dq_des - dq)
                self._mc.controlMIT(m, kp, kd, value, dq_des, tau_ff)
            elif data_name == "goal_velocity":
                # Velocity mode: kp=0 -> no position spring, only damping
                self._mc.controlMIT(m, 0.0, kd, 0.0, value, tau_ff)
            elif data_name == "goal_torque":
                # Open-loop torque: kp=0, kd=0, tau_ff=value
                self._mc.controlMIT(m, 0.0, 0.0, 0.0, 0.0, value)
            elif data_name == "goal_pos_vel":
                # Position-Speed mode (CAN ID 0x100+ID): two LE float32.
                # value = p_des (rad). dq_des is reused as v_des cruise cap.
                # Motor must have been switched to Control_Type.POS_VEL.
                self._mc.control_Pos_Vel(m, value, dq_des)
            elif data_name == "goal_speed":
                # Speed mode (CAN ID 0x200+ID): one LE float32 v_des.
                # Motor must have been switched to Control_Type.VEL.
                self._mc.control_Vel(m, value)
            else:   # "mit_command" -- full control, value is q_des
                self._mc.controlMIT(m, kp, kd, value, dq_des, tau_ff)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def motor_names(self) -> list[str]:
        """Return list of registered motor names in config order."""
        return list(self._motors)

    def switch_mode(self, mode: Control_Type,
                    names: list[str] | None = None) -> None:
        """Switch one or more motors to a different control mode.

        Required before calling write() with the matching data_name:

            Control_Type.MIT     -> goal_position / goal_velocity / goal_torque / mit_command
            Control_Type.POS_VEL -> goal_pos_vel
            Control_Type.VEL     -> goal_speed

        Args:
            mode:  Control_Type enum value.
            names: Motor names to switch.  Defaults to all motors.
        """
        targets = names if names is not None else list(self._motors)
        for name in targets:
            m = self._resolve(name)
            self._mc.switchControlMode(m, mode)
        time.sleep(0.05)

    def set_zero(self, names: list[str] | None = None) -> None:
        """Latch current shaft angle as 0 rad on one or more motors (volatile)."""
        targets = names if names is not None else list(self._motors)
        for name in targets:
            m = self._resolve(name)
            self._mc.set_zero_position(m)
        time.sleep(0.2)

    def refresh(self, names: list[str] | None = None) -> None:
        """Refresh cached feedback for one or more motors without returning it.

        Useful when you need q/dq/tau from get_raw_motor() afterwards.
        """
        targets = names if names is not None else list(self._motors)
        for name in targets:
            self._mc.refresh_motor_status(self._resolve(name))

    def get_raw_motor(self, name: str) -> Motor:
        """Return the underlying DM_CAN.Motor object for advanced use.

        Useful for calling DM_CAN-specific methods not exposed here,
        e.g. read_motor_param(), save_motor_param(), set_zero_position().
        """
        return self._resolve(name)

    @property
    def mc(self) -> MotorControl:
        """Expose the underlying MotorControl for advanced DM_CAN calls."""
        if self._mc is None:
            raise RuntimeError("DamiaoBus not connected. Call connect() first.")
        return self._mc

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _resolve(self, name: str) -> Motor:
        if name not in self._motors:
            raise KeyError(
                f"Motor '{name}' not found. "
                f"Available: {list(self._motors)}"
            )
        if self._mc is None:
            raise RuntimeError("DamiaoBus not connected. Call connect() first.")
        return self._motors[name]

    def _sync_limits(self) -> None:
        """Read PMAX/VMAX/TMAX from each motor and update Limit_Param."""
        for name, m in self._motors.items():
            cfg = self.config.motors[name]
            try:
                pmax = self._mc.read_motor_param(m, DM_variable.PMAX)
                vmax = self._mc.read_motor_param(m, DM_variable.VMAX)
                tmax = self._mc.read_motor_param(m, DM_variable.TMAX)
                if None not in (pmax, vmax, tmax):
                    self._mc.Limit_Param[int(cfg.model)] = [pmax, vmax, tmax]
                    print(
                        f"[DamiaoBus] {name}: limits synced "
                        f"PMAX={pmax} VMAX={vmax} TMAX={tmax}"
                    )
                else:
                    print(
                        f"[DamiaoBus] {name}: WARNING limit read returned None, "
                        f"using defaults from Limit_Param table"
                    )
            except Exception as exc:
                print(f"[DamiaoBus] {name}: WARNING limit sync failed: {exc}")
