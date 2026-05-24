# Copilot instructions -- exo-actuator-sensor-drivers workspace

This repo contains Python drivers and test scripts for hardware on a Jetson Orin Nano bench
**integrated with the PyDrake + Isaac Sim exosuit & manipulation framework**
(see parent [Isaac_sim_robotics](../../../) repo).

## Hardware

1. **Damiao DM-J4310-2EC V1.1** -- geared servo motor (24 V, CAN @ 1 Mbps,
   UART debug @ 921600 bps, 10:1 gearbox, ESC_ID=0x01).
2. **CubeMars AK60-6 V3.0 KV80** -- brushless servo motor (18–52 V, CAN @ 1 Mbps,
   extended-frame 29-bit CAN; SocketCAN `can1`, ESC_ID=0x02).
3. **CubeMars AK80-8 KV60 V1.x** -- brushless servo motor (24–48 V, CAN @ 1 Mbps,
   standard-frame 11-bit CAN; SocketCAN `can1`, ESC_ID=0x01).
4. **AS5048A** -- 14-bit absolute magnetic encoder (SPI mode 1, up to 10 MHz,
   Jetson SPI0 via device-tree overlay).

## Framework Integration

Use these notes when reasoning about, editing, or generating code in this workspace.
For actuator identification, hardware-to-sim methodology, and exosuit tuning, see:

- **Hardware-to-Sim Methodology**: `docs/notes/skyntefic_hardware_to_sim_methodology.tex`
  - Sharp pulse command identification (motor inertia & damping estimation)
  - Sinusoidal frequency-response tests (encoder-trace fitting to 2nd-order model)
  - Encoder comparison: hardware traces vs PyDrake simulation
  
- **SEA Tuning**: `Isaac_sim_robotics/.github/skills/sea-tuning/SKILL.md`
  - Cable-driven series elastic actuator spring stiffness & damping selection
  - Motor bandwidth tuning for tracking lag reduction
  
- **Exosuit Cable Routing**: `Isaac_sim_robotics/.github/instructions/exosuit-cables.instructions.md`
  - Dual-groove pulley antagonistic co-contraction design
  - Method A (offset pulleys) vs Method B (centred elbow pulley)
  - Effective stiffness: $K_{\mathrm{eff}} = 2\,k_{\mathrm{exo}}\,r_{\mathrm{exo}}^2$

- **Kinesthetic Teaching & Impedance Control**: `docs/notes/trajectory_control_cubemars.tex`
  - Low-impedance demos (Kp=0–10, Kd=0.3–0.8) for hand guiding
  - Trajectory recording & replay framework
  - Perturbation response measurement protocols

## Project layout
### Python source (`src/`)
**Root-level (shared by all motors):**
- `src/motor_config.py` -- **single source of truth** for hardware wiring.
  Dataclasses: `DamiaoMotorConfig`, `DamiaoBusConfig`, `CubeMarsMotorConfig`,
  `CubeMarsAkV3BusConfig`, `CubeMarsAkV1BusConfig`. Config instances:
  `DEFAULT_BENCH_CONFIG` (Damiao), `DEFAULT_AK60_6_BENCH_CONFIG` (AK60-6 V3.0),
  `DEFAULT_AK80_8_BENCH_CONFIG` (AK80-8 V1.x).
- `src/_common.py` -- test-script helpers:
  - `open_bus()` → `DamiaoBus` context manager (Damiao tests 03–10)
  - `open_cubemars_ak_v3_bench()` → `CubeMarsAkV3Bench` (AK60-6 V3.0 tests 01–07)
  - `open_cubemars_ak_v1_bench()` → `CubeMarsAkV1Bench` (AK80-8 V1.x tests)
  - `open_motor()` + `safe_disable_close()` (Damiao raw UART-only, tests 01–02)
- `src/as5048a.py` -- AS5048A encoder driver (context manager). Key methods:
  `read_angle_raw()`, `read_angle_deg()`, `read_diagnostics()`,
  `read_magnitude()`, `set_zero(burn_otp=False)`, `clear_error()`.
  Parity check uses full 16-bit even-parity (`_parity_ok()`).
  EF (error flag) is auto-cleared and retried once before raising.
- `src/encoder_config.py` -- `AS5048AConfig` dataclass + `DEFAULT_ENCODER_CONFIG`
  (`bus=0`, `device=0`, `max_hz=1_000_000`, `mode=1` → `/dev/spidev0.0`).

**Damiao subsystem (`src/damiao/`):**
- `src/damiao/DM_CAN.py` -- vendor driver (unchanged). Mirror at
  `old/motor-control-routine/Python例程/u2can/` -- keep both in sync if changing
  protocol behaviour.
- `src/damiao/damiao_bus.py` -- LeRobot-style `DamiaoBus` wrapper around DM_CAN.
  Exposes `connect()`, `disconnect()`, `is_connected`, `enable_torque()`,
  `disable_torque()`, `read()`, `write()`, `read_state()`, `set_zero()`,
  `switch_mode()`, `__enter__`/`__exit__`.

**CubeMars AK60-6 V3.0 subsystem (`src/cubemars/ak_v3/`):**
- `src/cubemars/ak_v3/AK_V3_CAN.py` -- CubeMars V3.0 driver. Class `CubeMarsAkV3Motor`
  (SocketCAN + python-can, extended 29-bit frames). Methods: `set_mit()`, `set_position_deg()`,
  `set_rpm()`, `set_duty()`, `set_current()`, `set_brake_current()`,
  `set_origin()`, `set_pos_spd()`, `parse_feedback()`, `poll_feedback()`.
- `src/cubemars/ak_v3/ak_v3_common.py` -- V3.0 constants and helpers: `CUBEMARS_AK_V3_LIMITS`
  table, `float_to_uint()`, `uint_to_float()`, packet-type constants.
- `src/cubemars/ak_v3/ak_v3_bus.py` -- `CubeMarsAkV3Bench` wrapper; mirrors `DamiaoBus` API.

**CubeMars AK80-8 V1.x subsystem (`src/cubemars/ak_v1/`):**
- `src/cubemars/ak_v1/AK_V1_CAN.py` -- CubeMars V1.x driver. Class `CubeMarsAkV1Motor`
  (SocketCAN, standard 11-bit frames, position-first MIT layout).
- `src/cubemars/ak_v1/ak_v1_bus.py` -- `CubeMarsAkV1Bench` wrapper; mirrors `DamiaoBus` API.

### Other directories
- `tests/damiao/` -- Damiao numbered test scripts 01–11 (UART/CAN control).
- `tests/cubemars/` -- CubeMars AK60-6 V3.0 test scripts (00_* probes + 01–07 control).
- `tests/cubemars-ak80-8-kv60/` -- CubeMars AK80-8 V1.x test scripts (00_* probes + 01–07 control).
- `tests/encoder/` -- AS5048A encoder test scripts 00–04 (SPI).
- `docs/datasheets/` -- Official datasheets (cubemars/, damaio/, encoder/, USB-to-CAN/).
- `docs/notes/dm_j4310_notes.tex` -- Damiao internal notes / audit. **Authoritative reference.**
  **Must be updated and recompiled after any structural change to `src/damiao/`, tests, or protocol.**
  Compile (twice): `cd docs/notes && pdflatex -interaction=nonstopmode dm_j4310_notes.tex`
- `docs/notes/cubemars_notes.tex` -- CubeMars (V3.0 + V1.x) internal notes / audit. **Authoritative reference.**
  **Must be updated and recompiled after changes to `src/cubemars/`, motor_config.py CubeMars sections, or tests.**
  Compile (twice): `cd docs/notes && pdflatex -interaction=nonstopmode cubemars_notes.tex`
- `docs/notes/as5048a_notes.tex` -- AS5048A encoder internal notes / audit. **Authoritative reference.**
  **Must be updated and recompiled after changes to `src/as5048a.py`, `src/encoder_config.py`, or `tests/encoder/`.**
  Compile (twice): `cd docs/notes && pdflatex -interaction=nonstopmode as5048a_notes.tex`
- `install_gs_usb.sh` -- Builds + installs `gs_usb.ko` out-of-tree on Tegra
  kernel. Required once on a fresh Jetson to enable DSDTech SH-C30A on can1.
- `test_can_adapter_dstech.sh` -- Verifies DSDTech adapter and SocketCAN loopback.
  Usage: `bash test_can_adapter_dstech.sh can1`
- `docs/notes/gs_usb_install.md` -- Full gs_usb installation guide.
- `old/motor-control-routine/` -- vendor examples for C, C#, MATLAB, ROS, STM32 (archived).
- `old/` -- archived root-level files (not active).
- `jetson-orin-spi-overlay-guide/` -- DTS overlay guide and install scripts.
- `jetson-orin-spi-overlay-guide/examples/as5048a-encoder/` -- overlay `.dts`,
  `install.sh`, and `README.md` for enabling SPI on the Jetson 40-pin header.

## Communication facts (do not redo from scratch)
- CAN std frame, 1 Mbps. Slave ID = `ESC_ID`. Master ID set on the motor.
- **MIT mode** CAN ID = `ESC_ID`; 8-byte bit-packed frame
  `[p15:8, p7:0, v11:4, v3:0|Kp11:8, Kp7:0, Kd11:4, Kd3:0|t11:8, t7:0]`.
  `Kp ∈ [0,500]`, `Kd ∈ [0,5]`. Position is 16-bit, vel/torque 12-bit, all
  scaled to firmware's `PMAX/VMAX/TMAX`.
- **Position-Speed mode** CAN ID = `0x100 + ESC_ID`; two LE float32:
  `p_des` (rad), `v_des` (rad/s, trapezoidal cruise speed).
- **Speed mode** CAN ID = `0x200 + ESC_ID`; one LE float32 `v_des` (rad/s).
- Enable/disable/zero use the special 8-byte frame
  `FF FF FF FF FF FF FF {FC|FD|FE}`.

## Critical gotchas (verified against this codebase)
1. **`Limit_Param` must match firmware `PMAX/VMAX/TMAX`.** There is no
   `DM_J4310_2EC` enum entry; users pick `DM_Motor_Type.DM4310` whose
   defaults are `[12.5, 30, 10]`. The J-series single-turn output usually
   ships with a much smaller `PMAX` (e.g. `pi`). Always read PMAX/VMAX/TMAX
   from the motor and overwrite `MotorControl.Limit_Param[motor_type]`
   before sending MIT commands -- otherwise the motor will go to the wrong
   position.
2. **Never set `Kd = 0` when commanding position in MIT mode** -- the motor
   will oscillate or runaway (PDF note).
3. `DM_CAN.py` historically used `time.sleep(...)` while only importing
   `from time import sleep`. The fix is `import time` at the top; keep both
   imports.
4. Driver is one-send-one-receive: `getPosition/Velocity/Torque` return
   cached values updated only after a control or `refresh_motor_status`
   call.
5. Power: 15-32 V; over-current cap 9.8 A; over-temperature trip 120 °C
   driver / ~100 °C motor.
6. **Firmware `Master_ID` must be `0x11`** (not the factory default `0x00`).
   With `Master_ID=0x00` the Python driver drops all feedback frames
   (filter mismatch) and `getPosition()` always returns 0, causing the
   motor to race to the wrong position. Set via Damiao Windows debug tool
   → Write Param.
7. **Output-shaft units only.** The firmware uses the output-shaft encoder
   directly. All CAN values (`p_des`, `v_des`, feedback) are already at the
   output shaft. Do NOT multiply by the 10:1 gear ratio.

## Control mode quick-reference
- **Position-Speed** (`0x100+ID`): two LE float32 `p_des` (rad), `v_des` (rad/s cap).
  Factory PID, trapezoidal ramp. Best for set-and-forget moves.
- **Speed** (`0x200+ID`): one LE float32 `v_des`. Continuous rotation.
- **MIT** (`ID`): bit-packed `q, dq, Kp, Kd, tau_ff`.
  - Position: Kp > 0, Kd > 0, q = target. **Never Kd=0.**
  - Velocity: Kp = 0, Kd > 0, dq = target.
  - Torque: Kp = 0, Kd = 0, tau_ff = desired torque. Open-loop.
- All values are **output-shaft units**. Firmware reads the output encoder directly.
  Do NOT multiply by gear ratio.

## Impedance control / kinesthetic teaching
To make the motor back-drivable by hand (for imitation learning / demonstration):
- Use MIT mode with **low Kp (0–10)** and **low Kd (0.3–0.8)**.
- Set `q_des = q_meas` each loop tick so the spring force stays zero.
- Record `(t, q, dq, tau)` at 100 Hz → `demo_trajectory.csv`.
- Replay with higher Kp (30–80) in `09_replay_trajectory.py`.
- See `docs/notes/dm_j4310_notes.tex` §Impedance control for the full recipe.
- Test scripts: `tests/damiao/08_kinesthetic_record.py`, `tests/damiao/09_replay_trajectory.py`.

## Test scripts (tests/damiao/)
| File | Mode | Purpose |
|---|---|---|
| 01_read_params.py  | UART   | Dump all firmware params |
| 02_write_params.py | UART   | Edit one param with confirmation |
| 03_pos_vel_control.py | Pos-Speed | Point-to-point move |
| 04_speed_control.py   | Speed     | Constant speed spin |
| 05_mit_position.py    | MIT Pos   | Kp/Kd position hold |
| 06_mit_velocity.py    | MIT Vel   | Velocity tracking |
| 07_mit_torque.py      | MIT Torque| Open-loop torque |
| 08_kinesthetic_record.py | MIT | Hand guiding + CSV record |
| 09_replay_trajectory.py  | MIT | Replay recorded demo |
| 10_impedance_spring_return.py | MIT | Spring return to origin |

Tests 01–02 use `open_motor()` + `safe_disable_close()` (raw driver, UART
read/write only). **Tests 03–10 use `with open_bus(...) as bus:` — never
call raw `mc.controlMIT()` / `mc.control_Pos_Vel()` directly in test scripts.**

- The Python driver mixes English/Chinese docstrings -- preserve both when
  editing.
- Do not introduce new dependencies; the driver uses only `numpy`,
  `pyserial`, `struct`, `time`.
- When changing CAN encoding, update **both** `DM_CAN.py` copies and the
  notes file, then recompile the PDF.

## LeRobot Bus API (tests 03–10)
All control test scripts use the context-manager idiom:

```python
from _common import open_bus, Control_Type

with open_bus(mode=Control_Type.POS_VEL, set_zero=True) as bus:
    bus.write("goal_pos_vel", {"j1": 1.57}, dq_des=5.0)
    q, dq, tau = bus.read_state()["j1"]   # one CAN round-trip for all three
```

**Import note:** `Control_Type` is re-exported from `_common.py` (line 43). Do NOT import
from `src/damiao/DM_CAN.py` directly in test scripts — always use `from _common import`.

Supported `write()` data names:
| data_name | Mode | Extra kwargs |
|---|---|---|
| `goal_position` | MIT | `kp`, `kd`, `dq_des`, `tau_ff` |
| `goal_velocity` | MIT | `kd` |
| `goal_torque`   | MIT | — |
| `mit_command`   | MIT | `kp`, `kd`, `dq_des`, `tau_ff` |
| `goal_pos_vel`  | POS_VEL | `dq_des` (cruise speed) |
| `goal_speed`    | VEL | — |

`read_state(names=None)` returns `dict[str, tuple[float,float,float]]`
`(q_rad, dq_rad_s, tau_Nm)` per named motor in **one** refresh per motor.

Both `DamiaoBus` and `CubeMarsAkV3Bench` and `CubeMarsAkV1Bench` implement the full contract:
`connect`, `disconnect`, `is_connected`, `enable_torque` (Damiao only), `disable_torque` (Damiao only),
`read`, `write`, `read_state`, `read_raw` (CubeMars only), `set_zero`, `__enter__`, `__exit__`.

---

# CubeMars AK60-6 V3.0 KV80

## Hardware facts
- Allowable voltage: 18–52 V. Rated 48 V, rated current 10 A, max 30 A.
- CAN bus: 1 Mbps, **CAN 2.0B extended frame** (29-bit IDs).
- Adapter: **DSDTech SH-C30A** (USB ID `1d50:606f`), candleLight/gs_usb family.
  Appears as `can1` after `gs_usb` kernel module is loaded (see `install_gs_usb.sh`).
- ESC_ID confirmed: **0x02** (set via R-Link; verify with `tests/cubemars/00_scan_can_id.py`).
- Feedback CAN ID: `(0x29 << 8) | ESC_ID` = `0x2902` for ESC=0x02.
- Calibration (Motor Identification + Encoder Identification) done once via
  R-Link before first use. Re-calibrate only after hardware reassembly or
  firmware update.

## V3.0 unified extended-ID scheme
- `CAN_ID = (PACKET_TYPE << 8) | ESC_ID`
- Packet types (from `ak_v3_common.py`):
  `0=DUTY, 1=CURRENT, 2=BRAKE, 3=RPM, 4=POS, 5=ORIGIN, 6=POS_SPD, 8=MIT`
- Feedback packet type: `0x29` (auto-broadcast or query-reply, per R-Link CAN Mode).

## V3.0 MIT byte order (Kp-first — different from V1.x)
```
byte0 = Kp[11:4]
byte1 = Kp[3:0] | Kd[11:8]
byte2 = Kd[7:0]
byte3 = pos[15:8]
byte4 = pos[7:0]
byte5 = vel[11:4]
byte6 = vel[3:0] | tau[11:8]
byte7 = tau[7:0]
```
Implemented in `CubeMarsAkV3Motor.set_mit()` — do not re-derive, just call the method.

## V3.0 Feedback frame (8 bytes, big-endian)
- bytes 0–1: int16 × 0.1 → degrees
- bytes 2–3: int16 × 10 → ERPM
- bytes 4–5: int16 × 0.01 → amps (Iq)
- byte 6: int8 MOS temperature (°C)
- byte 7: uint8 error code (0 = no fault)

## AK60-6 V3.0 KV80 parameter ranges (manual §4.2, verified)
`P_MAX = ±12.56 rad`, `V_MAX = ±60 rad/s`, `T_MAX = ±12 N·m`,
`Kp ∈ [0, 500]`, `Kd ∈ [0, 5]`.
Stored in `CUBEMARS_AK_V3_LIMITS["AK60-6"]` in `src/cubemars/ak_v3/ak_v3_common.py`.

## AK80-8 KV60 V1.x parameter ranges (tested)
`P_MAX = ±12.5 rad`, `V_MAX = ±37.5 rad/s`, `T_MAX = ±32 N·m`,
`Kp ∈ [0, 500]`, `Kd ∈ [0, 5]`.
Stored in `CUBEMARS_AK_V1_LIMITS["AK80-8"]` in `src/cubemars/ak_v1/ak_v1_can.py`.

## R-Link "CAN Mode" setting
`Periodic Feedback` = motor auto-broadcasts at CAN Fdb Rate.
`Query-Reply` (R-Link mistranslates as "Inquiry Feedback") = motor replies only
after a command. Both modes accept **all** packet types (MIT and servo commands)
simultaneously. This is feedback delivery only, not a mode-switch.

## CubeMars V3.0 Bus API (tests/cubemars/ 01–07)
```python
from _common import open_cubemars_ak_v3_bench

with open_cubemars_ak_v3_bench(set_zero=True) as bus:
    bus.write("goal_position", {"j1": 1.57}, kp=60, kd=1.5)
    q, dq, ia = bus.read_state()["j1"]   # q=rad, dq=rad/s, ia=amps(Iq)
```
`open_cubemars_ak_v3_bench()` opens SocketCAN `can1` and wraps `CubeMarsAkV3Motor(can_id=0x02)` (AK60-6 V3.0 only).
`read_state()` returns `(q_rad, dq_rad_s, current_a)`. No `mode` argument — MIT only.
`write()` supports the same `data_name` strings as `DamiaoBus`:

| data_name | Mode | Extra kwargs |
|---|---|---|
| `goal_position`     | MIT position | `kp`, `kd`, `dq_des`, `tau_ff` |
| `goal_velocity`     | MIT velocity | `kd`, `tau_ff` |
| `goal_torque`       | MIT open-loop torque | — |
| `mit_command`       | Raw MIT pass-through | `kp`, `kd`, `dq_des`, `tau_ff` |
| `goal_position_deg` | Servo position loop | — (degrees) |
| `goal_pos_spd`      | Servo pos-vel profile | `dq_des` (ERPM), `acc_erpm_s2` |
| `goal_speed_erpm`   | Servo speed loop | — |
| `goal_duty`         | Servo duty cycle | — |

`read_raw()` returns `(pos_deg, spd_erpm, current_a, temp_c, error_code)` for
debugging without SI unit conversion.

## Test scripts (tests/cubemars/)
| File | Purpose |
|---|---|
| 00_loopback_dsd_tech.py   | DSD TECH USB adapter loopback (software + physical) |
| 00_loopback_socketcan.py  | Jetson built-in CAN0 loopback (software + physical) |
| 00_scan_bitrate.py        | Passive bitrate scan (find motor's actual rate) |
| 00_probe_motor.py         | Safe zero-torque ping; confirms motor is alive |
| 00_sniff_motor.py         | Passive V3.0 feedback sniffer; no TX (safest first test) |
| 01_check_connection.py    | Poll 5 feedback frames, confirm non-zero response |
| 02_mit_position.py        | MIT position hold (Kp=60 Kd=1.5); sweep ±30° |
| 03_mit_velocity.py        | MIT velocity tracking (Kd=1.0) |
| 04_mit_torque.py          | Open-loop torque pulses (≤ 0.5 N·m unloaded) |
| 05_kinesthetic_record.py  | Hand-guide + save to `demo_trajectory_ak80.csv` |
| 06_replay_trajectory.py   | Replay CSV with Kp=30 Kd=1.0 |
| 07_impedance_spring_return.py | Soft spring anchored at zero |

All tests 01–07 use `with open_cubemars_ak_v3_bench() as bus:` — never call
`CubeMarsAkV3Motor` methods directly in test scripts.

## Adapter setup
1. Plug DSDTech SH-C30A into USB.
2. Ensure `gs_usb.ko` is loaded (one-time install via `bash install_gs_usb.sh`).
3. If can1 is not up: `sudo ip link set can1 up type can bitrate 1000000`
   (the udev rule in `install_gs_usb.sh` does this automatically on plug-in).
4. Verify: `bash test_can_adapter_dstech.sh can1`

## Critical gotchas (CubeMars)
### V3.0 (AK60-6)
1. **V3.0 firmware ignores standard 11-bit frames** entirely. The legacy
   `cubemars_bus.py` (HDSC serial transport) has been removed from the repo.
   **Use `src/cubemars/ak_v3/AK_V3_CAN.py` exclusively.**
2. **Never Kd=0 in position mode.** Same rule as Damiao and V1.x.
3. **ESC_ID = 0x02** (not 0x68). Wrong ID → `read_state()` returns cached zeros
   silently. Verify with `tests/cubemars/00_probe_motor.py` if unsure.
4. **Calibration required before first use.** Motor Identification + Encoder
   Identification via R-Link with motor unloaded. Re-calibrate after rewiring.
5. `demo_trajectory_ak80.csv` is the CubeMars recording file (separate from
   Damiao's `demo_trajectory.csv`).
6. **gs_usb "Unexpected unused echo id" errors** in dmesg → reload module:
   `sudo rmmod gs_usb && sudo modprobe gs_usb`
7. **can1 missing after reboot** → either replug the adapter (udev rule brings
   it up) or run: `sudo ip link set can1 up type can bitrate 1000000`
8. **`read_state()` tau field is amps (Iq), not N·m.** Multiply by motor Kt
   to get torque if needed.

### V1.x (AK80-8)
1. **Standard 11-bit CAN frames** (not extended 29-bit).
2. **Never Kd=0 in position mode.** Same rule as V3.0 and Damiao.
3. **ESC_ID = 0x01** (default). Verify with `tests/cubemars-ak80-8-kv60/00_probe_motor.py`.
4. **Calibration required before first use** via R-Link. Re-calibrate after rewiring.
5. **`read_state()` returns `(q_rad, dq_rad_s, ia_amps)`** (same as V3.0).

## docs/notes maintenance rule (CubeMars)
`docs/notes/cubemars_notes.tex` is the authoritative reference covering BOTH V3.0 and V1.x.
Update + recompile (twice) after any change to `src/cubemars/`, motor_config.py CubeMars sections,
or `tests/cubemars/` or `tests/cubemars-ak80-8-kv60/`:
```bash
cd docs/notes
pdflatex -interaction=nonstopmode cubemars_notes.tex
pdflatex -interaction=nonstopmode cubemars_notes.tex
```

---

# AS5048A Magnetic Encoder

## Hardware facts
- 14-bit absolute encoder, 0–16383 counts per revolution (360°).
- SPI mode 1 (CPOL=0, CPHA=1), MSB-first, up to 10 MHz. Bench: 1 MHz.
- Connected to Jetson Orin Nano SPI0 → `/dev/spidev0.0`.
  (`spi@3210000` enumerates as Linux `spi0` despite pad names `spi1_*_pz*`.)
- Device-tree overlay installed: `jetson-orin-as5048a.dtbo` in `/boot/`,
  activated via `extlinux.conf` `OVERLAYS` line.
- `spidev` Python package must be installed system-wide for root:
  `sudo pip3 install spidev` (NOT `--user`).

## SPI frame format
- 16-bit MSB-first frames. Command: `[PAR₁₅|RWn₁₄|addr₁₃..₀]`.
  Response (next frame): `[PAR₁₅|EF₁₄|data₁₃..₀]`.
- Parity: even over all 16 bits of the response frame (`_parity_ok()`).
- **One-frame latency**: send READ command, then send NOP -- data arrives in
  MISO of the NOP frame.
- Key registers: `0x3FFF` angle, `0x3FFD` diagnostics+AGC, `0x3FFE` magnitude,
  `0x0001` clear-error, `0x0016`/`0x0017` OTP zero offset.

## EF (error flag) handling
The EF bit (bit 14 of the response) is sticky. SPI noise on jumper wires
can set it spuriously. `read_register()` clears EF and retries once before
raising `RuntimeError`. If EF persists on both attempts, a real error is raised.

## Diagnostics register (0x3FFD)
- `OCF=1` → offset-compensation algorithm converged (needs real magnetic field).
  `OCF=0` without a magnet is normal -- not a bug.
- `COF=1` → CORDIC overflow, magnet too close/far.
- `COMP_LOW/COMP_HIGH` → AGC at limits (magnet too far / too close).
- `AGC` byte: 0 = very strong field, 255 = very weak. Target 50–200.

## Encoder config
```python
from encoder_config import DEFAULT_ENCODER_CONFIG  # bus=0, device=0, max_hz=1_000_000, mode=1
from as5048a import AS5048A

with AS5048A(DEFAULT_ENCODER_CONFIG) as enc:
    print(enc.read_angle_deg())       # 0..360 degrees
    print(enc.read_diagnostics())     # {'agc': 18, 'ocf': 1, 'cof': 0, ...}
    enc.set_zero(burn_otp=False)      # latch current pose as 0 (RAM, volatile)
```

## Test scripts (tests/encoder/)
| File | Purpose |
|---|---|
| 00_spi_connectivity.py | SPI loopback + raw chip probe; run first on new board |
| 01_read_angle.py       | 50 Hz continuous angle readout |
| 02_diagnostics.py      | AGC, magnitude, magnet placement health |
| 03_set_zero.py         | Latch current angle as zero (RAM only) |
| 04_zero_and_track.py   | Zero current position then track rotation with CW/CCW + total degrees |

Run all encoder tests with `sudo python3` (spidev requires root).

## Critical gotchas (encoder)
1. **Run as root.** `spidev` is not accessible without `sudo` on Jetson.
2. **Bus = 0, not 1.** `spi@3210000` maps to `spidev0.0`. Using `bus=1`
   opens the wrong controller and all reads return 0xFF.
3. **Stage 1 loopback fails with encoder connected.** That is expected --
   the encoder pulls MISO; `00_spi_connectivity.py` treats it as a warning
   not a failure.
4. **OTP burn is irreversible.** `set_zero(burn_otp=True)` programs the fuses
   exactly once. Only call when mechanical mounting is finalised.
5. Parity check is over the full 16-bit response (not bits 0..14).

## docs/notes maintenance rule (encoder)
After any change to `src/as5048a.py`, `src/encoder_config.py`, or
`tests/encoder/`, update `docs/notes/as5048a_notes.tex` and recompile:
```bash
cd docs/notes
pdflatex -interaction=nonstopmode as5048a_notes.tex
pdflatex -interaction=nonstopmode as5048a_notes.tex
```
Pre-existing non-fatal `\diameter` warnings on lines 50 and 441 are normal.

---

# docs/notes maintenance rule (Damiao)
After **any** of the following, update `docs/notes/dm_j4310_notes.tex` and
recompile:
- Adding or removing a file in `src/damiao/` or `tests/damiao/`
- Changing the `DamiaoBus` API (method names, data_names, kwargs)
- Changing the CAN protocol or frame encoding
- Adding a new motor type or control mode

Compile command (run twice from `docs/notes/` for cross-references):
```bash
pdflatex -interaction=nonstopmode dm_j4310_notes.tex
pdflatex -interaction=nonstopmode dm_j4310_notes.tex
```
Expected: `Output written on dm_j4310_notes.pdf (N pages, …)` with no `! `
error lines. Four pre-existing `\checkmark` undefined-control-sequence
warnings on lines 833–836 are non-fatal; PDF is still produced correctly.

## When asked about behaviour
1. **Damiao:** Quote `docs/notes/dm_j4310_notes.tex` PDF tables. Bit-packing is non-obvious; copy, do not re-derive.
2. **CubeMars:** Quote `docs/notes/cubemars_notes.tex` PDF tables. Covers both V3.0 and V1.x.
3. **Encoder:** Quote `docs/notes/as5048a_notes.tex` for SPI protocol and parity checks.
4. **Always check the authoritative `.tex` PDF before answering protocol/bit-packing questions** — don't guess.

---

# Hardware Experiment Protocols & Actuator Identification

## Kinesthetic Teaching (Impedance Control)
From `tests/damiao/08_kinesthetic_record.py` and `tests/cubemars/05_kinesthetic_record.py`:

```
Low impedance parameters (hand-guiding):
  Kp = 0–10 N⋅m/rad
  Kd = 0.3–0.8 N⋅m⋅s/rad
  q_des = q_meas (zero spring force)
  
Recording: (t, q, dq, τ) at 100 Hz → demo_trajectory.csv
Replay: higher Kp (30–80) to achieve trajectory tracking
```

**Use case:** imitation learning, demonstration capture for later RL training.

## Actuator Identification: Sharp Pulse + Sinusoidal Frequency Response

From `docs/notes/skyntefic_hardware_to_sim_methodology.tex`:

1. **Sharp pulse (impulse):** Apply brief step command (10–50 ms), record encoder response
   - Fit second-order underdamped model: $\omega_n$, $\zeta$, $\tau$ (delay)
   - Extract: motor inertia $J_m$, damping ratio $\zeta_m$, mechanical lag

2. **Sinusoidal sweep:** 0.1 Hz → 20 Hz, constant amplitude, record amplitude & phase lag
   - Compare encoder phase vs command: extracts frequency-dependent bandwidth limits
   - Fitting equation: $\phi(\omega) = -\arctan\left(\frac{2\zeta\omega_n\,\omega}{\omega_n^2 - \omega^2}\right) - \tau\,\omega$

3. **Cross-validation:** PyDrake simulation with identified $(J_m, \zeta_m)$ vs hardware traces
   - Adjust until PyDrake/hardware tracking errors match within 5%

**Typical fixtures:**
- Clamped joint (free-swinging, no load): motor inertia dominant
- Light mass load (50–200 g): adds virtual inertia to shaft
- Friction estimation: constant torque plateau in low-speed trials

## SEA Cable Dynamics Tuning

From `Isaac_sim_robotics/.github/skills/sea-tuning/SKILL.md`:

For cable-driven SEA (elbow joint):
- Spring stiffness $k_s$ ∈ [10, 100] N/m (softer = less tracking error, more settling time)
- Cable damping $b_c$ ∈ [1, 20] N⋅s/m (heavier damping reduces oscillation)
- Motor bandwidth mismatch → cable resonance → instability

**Tuning workflow:**
1. Start: $k_s = 30$ N/m, $b_c = 8$ N⋅s/m (default)
2. Measure: step response, frequency response, disturbance attenuation
3. If overshoot > 10%, increase $b_c$; if lag > 100 ms, decrease $k_s$

## Hardware-to-Sim Validation Loop

Integrate hardware measurements into the parent `Isaac_sim_robotics/` repo:

1. Record hardware traces: encoder + motor current/torque
2. Generate PyDrake plant model with identified inertia/damping parameters
3. Run identical trajectory in simulation with same controller gains
4. Compare end-effector error, joint accelerations, power consumption
5. If mismatch > 15%: re-identify or adjust friction model

**Output:** validated URDF + physics params for Onshape → simulation pipeline.

---

# docs/notes Maintenance Rules (Hardware Framework)

After changes to **any hardware drivers or test scripts**, update and recompile:

- `docs/notes/dm_j4310_notes.tex` (Damiao) — after changes to `src/damiao/` or `tests/damiao/`
- `docs/notes/cubemars_notes.tex` (CubeMars) — after changes to `src/cubemars/` or `tests/cubemars*/`
- `docs/notes/as5048a_notes.tex` (Encoder) — after changes to `src/as5048a.py` or `tests/encoder/`
- **New:** `docs/notes/skyntefic_hardware_to_sim_methodology.tex` — actuator ID methodology & results
- **New:** `docs/notes/trajectory_control_cubemars.tex` — kinesthetic teaching protocols & CSV formats

All `.tex` files must compile cleanly (run pdflatex twice from `docs/notes/`):

```bash
cd docs/notes
pdflatex -interaction=nonstopmode <filename>.tex
pdflatex -interaction=nonstopmode <filename>.tex
```

This repo is **not a standalone project** — it is a driver/hardware interface library
supporting the broader [Isaac_sim_robotics](../../../) dual-simulator framework for exosuit
trajectory control, cable-driven SEA, and parameter identification.
