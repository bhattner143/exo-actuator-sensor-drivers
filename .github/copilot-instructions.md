# Copilot instructions -- Damiao / CubeMars / AS5048A workspace

This repo contains Python drivers and test scripts for three hardware
components sharing one Jetson Orin Nano bench:

1. **Damiao DM-J4310-2EC V1.1** -- geared servo motor (24 V, CAN @ 1 Mbps,
   UART debug @ 921600 bps, 10:1 gearbox).
2. **CubeMars AK60-6 V3.0 KV80** -- brushless servo motor (18-52 V, CAN @ 1 Mbps,
   V3.0 firmware; controlled via `ak_v3_can.py` + `AkV3Bench` over SocketCAN
   `can1` (DSDTech SH-C30A, gs_usb); ESC_ID = 0x02).
3. **AS5048A** -- 14-bit absolute magnetic encoder (SPI mode 1, up to 10 MHz,
   connected to Jetson SPI0 via device-tree overlay).

Use these notes when reasoning about, editing, or generating code in this
workspace.

## Project layout
### Python source (`src/`)
- `src/DM_CAN.py` -- vendor driver (unchanged). A copy lives under
  `old/motor-control-routine/Python例程/u2can/` -- keep both in sync if changing
  protocol behaviour.
- `src/motor_config.py` -- **single source of truth** for hardware wiring.
  Contains `DamiaoMotorConfig`, `DamiaoBusConfig`, `CubeMarsMotorConfig`,
  `CubeMarsBusConfig`, `CUBEMARS_LIMITS`, `DEFAULT_BENCH_CONFIG` (Damiao bench:
  port `/dev/ttyACM0`, `can_id=0x01`, `master_id=0x11`), and
  `DEFAULT_CUBEMARS_BENCH_CONFIG` (CubeMars AK60-6 V3.0 KV80: port `/dev/ttyACM0`,
  `can_id=0x02` — legacy config, not used by V3.0 tests).
- `src/damiao_bus.py` -- LeRobot-style `DamiaoBus` wrapper around DM_CAN.
  Exposes `connect()`, `disconnect()`, `is_connected`, `enable_torque()`,
  `disable_torque()`, `read()`, `write()`, `read_state()`, `set_zero()`,
  `switch_mode()`, `__enter__`/`__exit__`.
- `src/cubemars_bus.py` -- **LEGACY** `CubemarsMotorsBus` (classic 11-bit HDSC
  serial transport). Does NOT work with V3.0 firmware (motor ignores standard
  11-bit frames). Kept for reference / V2.x compatibility.
- `src/ak_v3_can.py` -- **PRIMARY** CubeMars V3.0 driver. Class `AkV3Motor`
  (SocketCAN + python-can). Methods: `set_mit()`, `set_position_deg()`,
  `set_rpm()`, `set_duty()`, `set_current()`, `set_brake_current()`,
  `set_origin()`, `set_pos_spd()`, `parse_feedback()`, `poll_feedback()`.
  Uses `ak_v3_common.py` for bit-packing and limits.
- `src/ak_v3_common.py` -- Shared V3.0 constants and helpers: `AK_V3_LIMITS`
  table, `float_to_uint()`, `uint_to_float()`, packet-type constants.
- `src/ak_v3_serial.py` -- Experimental HDSC serial transport for V3.0
  (extended frames). Not used in primary tests.
- `src/_common.py` -- test-script helpers:
  - `open_bus()` → `DamiaoBus` context manager (Damiao tests 03–10)
  - `open_ak_v3_bench()` → `AkV3Bench` context manager (**primary CubeMars helper**)
  - `AkV3Bench` -- wraps `AkV3Motor` on SocketCAN `can1`; mirrors DamiaoBus API:
    `connect()`, `disconnect()`, `write()`, `read_state()`, `read_raw()`,
    `set_zero()`, `__enter__`/`__exit__`
  - `open_cubemars_bus()` → `CubemarsMotorsBus` (legacy; V3.0 hardware ignores it)
  - `open_motor()` + `safe_disable_close()` (Damiao raw driver, tests 01/02 only)
- `src/as5048a.py` -- AS5048A encoder driver (context manager). Key methods:
  `read_angle_raw()`, `read_angle_deg()`, `read_diagnostics()`,
  `read_magnitude()`, `set_zero(burn_otp=False)`, `clear_error()`.
  Parity check uses full 16-bit even-parity (`_parity_ok()`).
  EF (error flag) is auto-cleared and retried once before raising.
- `src/encoder_config.py` -- `AS5048AConfig` dataclass + `DEFAULT_ENCODER_CONFIG`
  (`bus=0`, `device=0`, `max_hz=1_000_000`, `mode=1` → `/dev/spidev0.0`).

### Other directories
- `tests/damiao/` -- Damiao numbered test scripts 01–10 (see table below).
- `tests/cubemars/` -- CubeMars AK-series test scripts (00_* probes + 01–07 V3.0 tests).
- `tests/encoder/` -- AS5048A encoder test scripts 00–04.
- `docs/datasheets/` -- Official datasheets (cubemars/, damaio/, encoder/, USB-to-CAN/).
- `docs/notes/dm_j4310_notes.tex` -- Damiao internal notes / audit. Read first
  when asked anything about the Damiao motor. **Must be updated and recompiled
  after any structural change to the repo or protocol** (see maintenance rule).
- `docs/notes/as5048a_notes.tex` -- AS5048A encoder internal notes / audit.
  Read first when asked anything about the encoder. **Must be updated and
  recompiled after changes to `src/as5048a.py`, `src/encoder_config.py`, or
  `tests/encoder/`** (see maintenance rule).
- `docs/notes/cubemars_notes.tex` -- CubeMars notes file. Read first when asked
  anything about the CubeMars motor. **Must be updated and recompiled after
  changes to `src/ak_v3_can.py`, `src/ak_v3_common.py`, `src/motor_config.py`
  (CubeMars sections), or `tests/cubemars/`.** Compile command (twice):
  `cd docs/notes && pdflatex -interaction=nonstopmode cubemars_notes.tex`
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
from _common import open_bus
from DM_CAN import Control_Type

with open_bus(mode=Control_Type.POS_VEL, set_zero=True) as bus:
    bus.write("goal_pos_vel", {"j1": 1.57}, dq_des=5.0)
    q, dq, tau = bus.read_state()["j1"]   # one CAN round-trip for all three
```

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

Both `DamiaoBus` and `CubemarsMotorsBus` implement the full contract:
`connect`, `disconnect`, `is_connected`, `enable_torque`, `disable_torque`,
`read`, `write`, `read_state`, `set_zero`, `__enter__`, `__exit__`.

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

## V3.0 MIT byte order (Kp-first — different from Damiao/classic CubeMars)
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
Implemented in `AkV3Motor.set_mit()` — do not re-derive, just call the method.

## V3.0 Feedback frame (8 bytes, big-endian)
- bytes 0–1: int16 × 0.1 → degrees
- bytes 2–3: int16 × 10 → ERPM
- bytes 4–5: int16 × 0.01 → amps (Iq)
- byte 6: int8 MOS temperature (°C)
- byte 7: uint8 error code (0 = no fault)

## AK60-6 V3.0 KV80 parameter ranges (manual §4.2, verified)
`P_MAX = ±12.56 rad`, `V_MAX = ±60 rad/s`, `T_MAX = ±12 N·m`,
`Kp ∈ [0, 500]`, `Kd ∈ [0, 5]`.
These are stored in `AK_V3_LIMITS["AK60-6"]` in `src/ak_v3_common.py`.

## R-Link "CAN Mode" setting
`Periodic Feedback` = motor auto-broadcasts at CAN Fdb Rate.
`Query-Reply` (R-Link mistranslates as "Inquiry Feedback") = motor replies only
after a command. Both modes accept **all** packet types (MIT and servo commands)
simultaneously. This is feedback delivery only, not a mode-switch.

## CubeMars V3.0 Bus API (tests/cubemars/ 01–07)
```python
from _common import open_ak_v3_bench

with open_ak_v3_bench(set_zero=True) as bus:
    bus.write("goal_position", {"j1": 1.57}, kp=60, kd=1.5)
    q, dq, ia = bus.read_state()["j1"]   # q=rad, dq=rad/s, ia=amps(Iq)
```
`open_ak_v3_bench()` opens SocketCAN `can1` and wraps `AkV3Motor(can_id=0x02)`.
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

All tests 01–07 use `with open_ak_v3_bench() as bus:` — never call
`AkV3Motor` methods directly in test scripts.

## Adapter setup
1. Plug DSDTech SH-C30A into USB.
2. Ensure `gs_usb.ko` is loaded (one-time install via `bash install_gs_usb.sh`).
3. If can1 is not up: `sudo ip link set can1 up type can bitrate 1000000`
   (the udev rule in `install_gs_usb.sh` does this automatically on plug-in).
4. Verify: `bash test_can_adapter_dstech.sh can1`

## Critical gotchas (CubeMars V3.0)
1. **V3.0 firmware ignores standard 11-bit frames** entirely. The legacy
   `cubemars_bus.py` (HDSC serial transport) will NOT work. Use `ak_v3_can.py`.
   Legacy test files (`00_sanity_check_legacy.py`, `00_scan_can_id.py`,
   `00_probe_v3_serial.py`) have been removed from the repo.
2. **Never Kd=0 in position mode.** Same rule as Damiao.
3. **ESC_ID = 0x02** (not 0x68). Wrong ID → `read_state()` returns cached zeros
   silently. Verify with `00_scan_can_id.py` if unsure.
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

## docs/notes maintenance rule (CubeMars)
`docs/notes/cubemars_notes.tex` is the authoritative reference. Update +
recompile (twice) after any change to `src/ak_v3_can.py`, `src/ak_v3_common.py`,
the `motor_config.py` CubeMars sections, or `tests/cubemars/`:
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
- Adding or removing a file in `src/` or `tests/`
- Changing the bus API (method names, data_names, kwargs)
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
Quote `docs/notes/dm_j4310_notes.tex` and the PDF tables rather than
guessing. Bit-packing layouts are non-obvious; copy the table, do not
re-derive.
