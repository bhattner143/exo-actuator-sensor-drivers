# CubeMars AK-Series V3.0 Protocol Reference

Distilled from **AK Series Module Product Manual V3.0.1** and verified
against the official `AK-V3.ino` Arduino example.

> ⚠️ This document describes the **V3.0 driver-board firmware** protocol.
> It is **different** from the classic mini-cheetah MIT protocol used by
> Damiao and earlier T-Motor firmware (and by `src/cubemars_bus.py`).
> See [`src/ak_v3_can.py`](../../src/ak_v3_can.py) for the Python port.

## 1. Physical layer

| Field | Value |
|---|---|
| CAN bitrate | 1 Mbps |
| Frame format | CAN 2.0B **extended** (29-bit ID) |
| DLC | 8 bytes |
| Termination | 120 Ω across CAN_H / CAN_L at bus ends |
| Power | 18–52 V (rated 48 V) |

## 2. Identifier scheme — unified for all modes

```
CAN_ID (29-bit) = (PACKET_TYPE << 8) | ESC_ID
```

`PACKET_TYPE` enum (manual §4.1, §4.2):

| Type | Name | Mode | CAN_ID for ESC=0x68 |
|---|---|---|---|
| 0 | `SET_DUTY`           | Duty cycle           | `0x00000068` |
| 1 | `SET_CURRENT`        | Current loop (Iq)    | `0x00000168` |
| 2 | `SET_CURRENT_BRAKE`  | Current brake        | `0x00000268` |
| 3 | `SET_RPM`            | Velocity loop (ERPM) | `0x00000368` |
| 4 | `SET_POS`            | Position loop        | `0x00000468` |
| 5 | `SET_ORIGIN_HERE`    | Set origin           | `0x00000568` |
| 6 | `SET_POS_SPD`        | Position-Velocity    | `0x00000668` |
| **8** | **`SET_MIT`**    | **Force control (MIT)** | **`0x00000868`** |

## 3. R-Link `CAN Mode` setting (feedback delivery only)

| Setting | Meaning |
|---|---|
| **Inquiry Feedback**  | Motor replies once for each command received (poll/response). |
| **Periodic Feedback** | Motor broadcasts feedback at `CAN Fdb Rate` Hz autonomously. |

Both Servo and MIT commands work in either setting. The mode is **not**
selected by this dropdown.

## 4. Force-control (MIT) frame — V3.0 byte order

```
buffer[0] = kp_int >> 4                          // Kp high 8 bits
buffer[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8)// Kp low 4 | Kd high 4
buffer[2] = kd_int & 0xFF                        // Kd low 8 bits
buffer[3] = p_int >> 8                           // position high 8
buffer[4] = p_int & 0xFF                         // position low 8
buffer[5] = v_int >> 4                           // velocity high 8
buffer[6] = ((v_int & 0xF) << 4) | (t_int >> 8)  // vel low 4 | tau high 4
buffer[7] = t_int & 0xFF                         // torque low 8
```

> **This is Kp-first**, not position-first. The classic mini-cheetah
> layout (used by Damiao and `cubemars_bus.py`) puts position first.

### Sub-modes (selected by which fields are nonzero)

| Sub-mode | Kp | Kd | Active field |
|---|---|---|---|
| Position | > 0 | > 0 | `p_des` |
| Velocity | 0   | > 0 | `v_des` |
| Torque   | 0   | 0   | `t_ff`  |

**Never use Kd = 0 in position mode** — the motor will oscillate.

## 5. Parameter ranges (manual §4.2 table, p. 42)

| Model | KV | P_max (rad) | V_max (rad/s) | T_max (N·m) | Kp | Kd |
|---|---|---|---|---|---|---|
| AK10-9  | 60  | ±12.56 | ±28 | ±54 | 0–500 | 0–5 |
| **AK60-6** | **80** | **±12.56** | **±60** | **±12** | **0–500** | **0–5** |
| AK70-9  | 60  | ±12.56 | ±30 | ±32 | 0–500 | 0–5 |
| AK80-9  | 100 | ±12.56 | ±65 | ±18 | 0–500 | 0–5 |
| AKE60-8 | 80  | ±12.56 | ±40 | ±15 | 0–500 | 0–5 |
| AKE80-8 | 30  | ±12.56 | ±20 | ±35 | 0–500 | 0–5 |

> The bench **AK60-6 V3.0 KV80** uses V_max = 60, T_max = 12. These
> differ from TMotorCANControl defaults (50, 15) and the older Damiao
> assumption (45, 15).

## 6. Feedback frame

Reply ID: `(0x29 << 8) | ESC_ID` → `0x2968` for ESC=0x68.

| Bytes | Type | Decoded value |
|---|---|---|
| 0–1 | int16 | position × 0.1 → degrees |
| 2–3 | int16 | speed × 10 → ERPM |
| 4–5 | int16 | current × 0.01 → amps |
| 6   | int8  | MOS temperature (°C) |
| 7   | uint8 | error code (0 = no fault) |

### Error codes (manual §4.3.1)

| Code | Meaning |
|---|---|
| 0 | No fault |
| 1 | Over-voltage |
| 2 | Under-voltage |
| 3 | Driver fault |
| 4 | Motor over-current |
| 5 | MOS over-temperature |
| 6 | Motor over-temperature |
| 7+ | See manual table |

## 7. Servo-mode commands (manual §4.1)

| Mode | Payload | Scale |
|---|---|---|
| Duty cycle | int32 BE | `(duty × 100000)` |
| Current    | int32 BE | `(amps × 1000)` |
| Brake      | int32 BE | `(amps × 1000)` |
| RPM        | int32 BE | `int(erpm)` |
| Position   | int32 BE | `(degrees × 10000)` |
| Set origin | uint8    | `0` = temporary, `1` = flash (permanent) |
| Pos+spd    | int32 + int16 + int16 BE | pos×10000, spd÷10, acc÷10 |

## 8. Hardware

Use a **SocketCAN-compatible USB-to-CAN adapter** (Waveshare USB-CAN-A
or DSD TECH Canable / SH-C31A). Linux brings up `can0` automatically
via the `gs_usb` driver. Before first use:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

## 9. Python usage

```python
import can
from ak_v3_can import AkV3Motor, ensure_can0_up

ensure_can0_up(1_000_000)
bus = can.interface.Bus(channel="can0", bustype="socketcan")
motor = AkV3Motor(bus, can_id=104, model="AK60-6")   # 104 = 0x68

# Hold 90° with moderate impedance
motor.set_mit(p_des=1.57, v_des=0.0, kp=30.0, kd=1.0, t_ff=0.0)

while True:
    if motor.poll_feedback(timeout=0.05):
        print(motor.pos_deg, motor.spd_erpm, motor.current_a)
```

## 10. Calibration (one-time, after assembly or firmware flash)

Use R-Link upper computer (CubeMars AK Config UI V3.x):

1. Connect via COM port, baud 921600.
2. **Read** firmware parameters (must succeed first).
3. **Motor Identification** — short beep then 10 s spin under no load.
4. **Encoder Identification** — slow rotation for ~45 s.
5. **Write** to persist `.AppParams` / `.McParams`.

Repeat only after rewiring phase leads or flashing new firmware.

## 11. Differences from the legacy MIT protocol used in this repo

| Aspect | Legacy (Damiao / `cubemars_bus.py`) | V3.0 firmware (this doc) |
|---|---|---|
| CAN frame type | Standard 11-bit ID | **Extended 29-bit ID** |
| Command CAN ID | `ESC_ID` | **`(PACKET_TYPE << 8) \| ESC_ID`** |
| MIT byte order | Position first | **Kp first** |
| Feedback CAN ID | `ESC_ID` (or `master_id`) | **`0x2900 \| ESC_ID`** |
| Feedback content | Bit-packed `q/dq/tau` | Plain int16 `pos/spd/cur` + temp + error |
| Transport on bench | HDSC USB-CAN (proprietary 30-byte serial envelope) | SocketCAN via `gs_usb` |

## References

- `docs/datasheets/cubemars/AK Series Module Product Manual V3.0.1`
- `AK-V3.ino` — official CubeMars Arduino example (MCP2515)
- `src/ak_v3_can.py` — Python port using python-can/SocketCAN
