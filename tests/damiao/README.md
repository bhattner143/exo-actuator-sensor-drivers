# Damiao DM-J4310-2EC test scripts

Clean, single-purpose scripts for the Damiao DM-J4310-2EC V1.1 motor.
Each file demonstrates **one** mode and uses [`_common.py`](_common.py)
for the boilerplate (port, IDs, gear-ratio helpers, limit sync).

## Setup once

1. Plug the HDSC USB-to-CAN adapter — appears as `/dev/ttyACM0`.
2. Power the motor with **24 V**, ensure the green LED comes on after
   `enable()`.
3. Edit `_common.py` if your `CAN_ID`, `MASTER_ID`, or serial port differ.
4. Make sure the **firmware Master ID is `0x11`** (or whatever you put in
   `_common.py`). It must NOT be `0x00`. Use the Damiao Windows debug
   tool's *Set Parameters → Master ID* field, then *Write Param*.

## Files

| Script | Mode | CAN ID used | What it does |
|---|---|---|---|
| [01_read_params.py](01_read_params.py)  | UART   | n/a              | Dumps PMAX/VMAX/TMAX, GR, IDs, gains, etc. |
| [02_write_params.py](02_write_params.py)| UART   | n/a              | Edits a single parameter (with prompt). |
| [03_pos_vel_control.py](03_pos_vel_control.py) | Pos-Speed | `0x100+ID` | Trapezoidal move to target position. |
| [04_speed_control.py](04_speed_control.py)     | Speed     | `0x200+ID` | Spin at constant rad/s. |
| [05_mit_position.py](05_mit_position.py)       | MIT       | `ID`       | Position control with Kp/Kd (Kd>0 !). |
| [06_mit_velocity.py](06_mit_velocity.py)       | MIT       | `ID`       | Velocity-only (Kp=0, Kd>0). |
| [07_mit_torque.py](07_mit_torque.py)           | MIT       | `ID`       | Open-loop torque (Kp=0, Kd=0, t_ff). |

## Gearbox note (very important)

The motor has a 10:1 gearbox but the firmware ships with `Gear factor = 1`,
so all CAN-side `p_des / v_des / t_ff` refer to the **rotor side**, not
the output shaft.

* Output angle = `p_des / 10`
* Output speed = `v_des / 10`
* Output torque ≈ `t_ff * 10` (minus efficiency losses)

The helpers `rad_out_to_motor()` / `rad_motor_to_out()` in `_common.py`
make this conversion explicit.

## Critical safety rules (from the PDF)

* **Never set `Kd = 0` when commanding a position in MIT mode** — the
  motor will oscillate or run away.
* If a position command goes to the wrong place, check (in order):
  1. firmware `Master ID` ≠ `0x00`,
  2. firmware `PMAX` matches `Limit_Param[motor_type][0]`,
  3. you're accounting for the **10:1 gearbox**.
* Send commands at ≥ 10 Hz; otherwise the firmware comm-loss timeout
  disables the motor.
