#!/usr/bin/env python3
"""tests/cubemars/00_read_params.py

Read all motor operating parameters from the CubeMars AK60-6 V3.0
over its **serial UART interface** (not CAN).

Uses COMM_GET_VALUES (command 0x45, §4.3.2.1 of the V3.0.1 manual).
The motor replies with a 74-byte framed packet containing:

  MOS temperature, motor temperature, output current, input current,
  Id current, Iq current, duty cycle, motor speed (ERPM), input voltage,
  [24 bytes reserved], motor status code, outer-loop position (rad),
  motor ID, [6 bytes reserved], Vd voltage, Vq voltage,
  current control mode, encoder angle, outer encoder angle.

Also issues COMM_GET_VALUES_SETUP (command 0x10) to query individual
parameters via bitmask, demonstrating selective parameter reads.

HARDWARE SETUP
--------------
  Motor UART : 3-pin terminal on the AK60-6 driver board (§1.2.2)
               Pin 1 = GND (Black), Pin 2 = RX (Yellow), Pin 3 = TX (Green)
  Adapter    : Any USB-to-UART (FTDI/CH340 etc.) at 921600 baud, 8N1
               OR the CubeMars R-Link dongle (USB -> 8-pin -> UART 3-pin)
  Device     : /dev/ttyUSB0 by default (change PORT below if different)

IMPORTANT
---------
  - This script uses the UART serial interface, NOT the DSDTech CAN adapter.
  - Motor does NOT need to be enabled / powered to its control voltage --
    it only needs logic power (via the VCC pin on the R-Link).
  - No motor motion will occur (read-only commands).

RUN
---
  sudo python3 tests/cubemars/00_read_params.py
  sudo python3 tests/cubemars/00_read_params.py --port /dev/ttyUSB1
"""
from __future__ import annotations

import sys
import os
import time
import struct
import argparse

import serial

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"),
)

# ---------------------------------------------------------------------------
# Serial port defaults
# ---------------------------------------------------------------------------
PORT    = "/dev/ttyUSB0"   # Change if using a different USB-UART adapter
BAUD    = 921_600
TIMEOUT = 0.5              # seconds to wait for a reply

# ---------------------------------------------------------------------------
# Protocol constants (§4.3.2)
# ---------------------------------------------------------------------------
FRAME_HEADER = 0xAA
FRAME_TAIL   = 0xBB

COMM_GET_VALUES       = 0x45   # 69  -- full status dump
COMM_GET_VALUES_SETUP = 0x10   # 16  -- selective bitmask query

# Bitmask bits for COMM_GET_VALUES_SETUP (LSB = bit 1)
# Bits as documented in §4.3.2.1 table (starting from bit 1):
#   1  MOS temp (2B)       2  Motor temp (2B)
#   3  Output current (4B) 4  Input current (4B)
#   5  Iq current (4B)     6  Id current (4B)
#   7  Duty cycle (2B)     8  Motor speed (4B)
#   9  Input voltage (2B) 10-15 Reserved
#  16  Motor error (1B)   17  Motor position (4B)  18  Motor ID (1B)
BITMASK_MOS_TEMP     = 1 << 0
BITMASK_MOTOR_TEMP   = 1 << 1
BITMASK_OUTPUT_CUR   = 1 << 2
BITMASK_INPUT_CUR    = 1 << 3
BITMASK_IQ_CUR       = 1 << 4
BITMASK_ID_CUR       = 1 << 5
BITMASK_DUTY         = 1 << 6
BITMASK_SPEED        = 1 << 7
BITMASK_VOLTAGE      = 1 << 8
BITMASK_ERROR        = 1 << 15
BITMASK_POSITION     = 1 << 16
BITMASK_MOTOR_ID     = 1 << 17

# ---------------------------------------------------------------------------
# CRC-16 (§4.3.2.3)
# ---------------------------------------------------------------------------
_CRC16_TAB = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
]


def crc16(data: bytes) -> int:
    """CRC-16 as specified in §4.3.2.3 of the V3.0.1 manual."""
    cksum = 0
    for b in data:
        cksum = _CRC16_TAB[((cksum >> 8) ^ b) & 0xFF] ^ ((cksum << 8) & 0xFFFF)
    return cksum


# ---------------------------------------------------------------------------
# Frame builders
# ---------------------------------------------------------------------------

def build_frame(comm_id: int, payload: bytes = b"") -> bytes:
    """Build a serial frame: AA <len> <comm_id> [payload] <crc_hi> <crc_lo> BB."""
    data = bytes([comm_id]) + payload
    length = len(data)
    crc = crc16(data)
    return bytes([FRAME_HEADER, length]) + data + bytes([crc >> 8, crc & 0xFF, FRAME_TAIL])


def build_get_values() -> bytes:
    """COMM_GET_VALUES (0x45): request full motor status dump."""
    return build_frame(COMM_GET_VALUES)


def build_get_values_setup(mask: int) -> bytes:
    """COMM_GET_VALUES_SETUP (0x10): request selected parameters by bitmask."""
    payload = struct.pack(">I", mask)   # 4-byte big-endian mask
    return build_frame(COMM_GET_VALUES_SETUP, payload)


# ---------------------------------------------------------------------------
# Frame receiver
# ---------------------------------------------------------------------------

def recv_frame(ser: serial.Serial, timeout: float = TIMEOUT) -> bytes | None:
    """Read bytes until a complete 0xAA...0xBB frame is found, or timeout."""
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
        # Try to extract a complete frame from buf
        while len(buf) >= 6:
            try:
                start = buf.index(FRAME_HEADER)
            except ValueError:
                buf.clear()
                break
            if start > 0:
                del buf[:start]
            if len(buf) < 3:
                break
            length = buf[1]
            total = 2 + length + 2 + 1   # header + len + data + crc(2) + tail
            if len(buf) < total:
                break
            if buf[total - 1] != FRAME_TAIL:
                del buf[0]   # not a valid frame start, skip
                continue
            frame = bytes(buf[:total])
            del buf[:total]
            # Verify CRC
            data_section = frame[2 : 2 + length]
            expected_crc = crc16(data_section)
            got_crc = (frame[2 + length] << 8) | frame[2 + length + 1]
            if expected_crc != got_crc:
                print(f"  [warn] CRC mismatch: expected 0x{expected_crc:04X}, "
                      f"got 0x{got_crc:04X}")
                continue
            return data_section   # strip header/len/crc/tail, return payload
    return None


# ---------------------------------------------------------------------------
# Response parsers  (§4.3.2.1)
# ---------------------------------------------------------------------------

_ERROR_CODES = {
    0: "No fault",
    1: "Motor over-temperature",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temperature",
    7: "Motor lock-up",
}


def parse_get_values(payload: bytes) -> dict:
    """Parse a COMM_GET_VALUES (0x45) reply.

    The reply data field starts with comm_id byte 0x45, then the payload.
    Byte layout (after stripping the frame wrapper, indices into payload):
      [0]      comm_id (0x45) -- already stripped, this is the data section
    Actually recv_frame() returns the full data section including comm_id.
    Layout from §4.3.2.1:
      0        comm_id 0x45
      1-2      MOS temp       int16  / 10.0 => °C
      3-4      Motor temp     int16  / 10.0 => °C
      5-8      Output current int32  / 100.0 => A
      9-12     Input current  int32  / 100.0 => A
      13-16    Id current     int32  / 100.0 => A
      17-20    Iq current     int32  / 100.0 => A
      21-22    Duty cycle     int16  / 1000.0
      23-26    Motor speed    int32  (ERPM)
      27-28    Input voltage  int16  / 10.0 => V
      29-52    Reserved       24 bytes
      53       Motor status code  uint8
      54-57    Outer-loop position  float (IEEE-754)
      58       Motor ID            uint8
      59-64    Reserved       6 bytes
      65-68    Vd voltage     int32  / 1000.0 => V
      69-72    Vq voltage     int32  / 1000.0 => V
      73-76    Current control mode  int32
      77-80    Encoder angle  float
      81-84    Outer encoder angle  float
    """
    if not payload or payload[0] != COMM_GET_VALUES:
        return {}
    d = payload[1:]   # strip comm_id byte
    if len(d) < 84:
        return {"_raw_len": len(d), "_raw": payload.hex()}

    idx = 0
    def i16(): nonlocal idx; v = struct.unpack_from(">h", d, idx)[0]; idx += 2; return v
    def i32(): nonlocal idx; v = struct.unpack_from(">i", d, idx)[0]; idx += 4; return v
    def u8():  nonlocal idx; v = d[idx]; idx += 1; return v
    def f32(): nonlocal idx; v = struct.unpack_from(">f", d, idx)[0]; idx += 4; return v

    mos_temp    = i16() / 10.0
    motor_temp  = i16() / 10.0
    out_cur     = i32() / 100.0
    in_cur      = i32() / 100.0
    id_cur      = i32() / 100.0
    iq_cur      = i32() / 100.0
    duty        = i16() / 1000.0
    speed_erpm  = i32()
    voltage     = i16() / 10.0
    idx += 24   # skip 24 reserved bytes
    status_code = u8()
    position    = f32()
    motor_id    = u8()
    idx += 6    # skip 6 reserved bytes
    vd          = i32() / 1000.0
    vq          = i32() / 1000.0
    ctrl_mode   = i32()
    enc_angle   = f32()
    outer_enc   = f32()

    return {
        "mos_temp_C":       mos_temp,
        "motor_temp_C":     motor_temp,
        "output_current_A": out_cur,
        "input_current_A":  in_cur,
        "id_current_A":     id_cur,
        "iq_current_A":     iq_cur,
        "duty_cycle":       duty,
        "speed_erpm":       speed_erpm,
        "input_voltage_V":  voltage,
        "status_code":      status_code,
        "position_rad":     position,
        "motor_id":         motor_id,
        "vd_V":             vd,
        "vq_V":             vq,
        "ctrl_mode":        ctrl_mode,
        "encoder_angle_deg": enc_angle,
        "outer_encoder_deg": outer_enc,
    }


def parse_get_values_setup(payload: bytes) -> list[tuple[str, float | int]]:
    """Parse a COMM_GET_VALUES_SETUP (0x10/0x13) selective reply.

    The motor only includes fields whose bitmask bit was set, in a fixed
    order (MOS temp first, outer encoder last).  We identify the fields
    by the comm_id byte and parse whatever is present.
    """
    if not payload or payload[0] != COMM_GET_VALUES_SETUP:
        return []
    # The reply mirrors the requested fields in fixed order.
    # Without knowing the exact mask we sent, we parse conservatively:
    # just display hex + known scalars.
    return [("raw_hex", payload[1:].hex())]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def print_section(title: str) -> None:
    print()
    print(f"{'=' * 55}")
    print(f"  {title}")
    print(f"{'=' * 55}")


def run(port: str) -> None:
    print(f"CubeMars AK60-6 V3.0 -- Serial parameter read")
    print(f"Port: {port}  Baud: {BAUD}")

    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as exc:
        print(f"\nFAIL: Cannot open {port}: {exc}")
        print("  → Is the USB-UART adapter connected to the motor's UART pins?")
        print("  → Run as sudo?  Check: ls -l /dev/ttyUSB*")
        sys.exit(1)

    time.sleep(0.1)
    ser.reset_input_buffer()

    # ------------------------------------------------------------------
    # 1. COMM_GET_VALUES -- full status dump
    # ------------------------------------------------------------------
    print_section("COMM_GET_VALUES (0x45) -- Full status")

    cmd = build_get_values()
    print(f"  TX: {cmd.hex().upper()}")
    ser.write(cmd)

    payload = recv_frame(ser, timeout=TIMEOUT)
    if payload is None:
        print("  FAIL: No response received within timeout.")
        print("  → Is the motor powered?")
        print("  → Verify UART wiring: Jetson TX → Motor RX, Motor TX → Jetson RX")
        print("  → Check baud rate (should be 921600).")
        ser.close()
        sys.exit(1)

    print(f"  RX: {payload.hex().upper()}")
    params = parse_get_values(payload)

    if not params:
        print(f"  FAIL: Unexpected response (comm_id=0x{payload[0]:02X} "
              f"!= 0x{COMM_GET_VALUES:02X})")
        ser.close()
        sys.exit(1)

    print()
    print(f"  {'Parameter':<25s}  {'Value':>12s}  Unit")
    print(f"  {'-'*52}")

    rows = [
        ("MOS temperature",     params["mos_temp_C"],        "°C"),
        ("Motor temperature",   params["motor_temp_C"],       "°C"),
        ("Output current",      params["output_current_A"],   "A"),
        ("Input current",       params["input_current_A"],    "A"),
        ("Id current",          params["id_current_A"],       "A"),
        ("Iq current",          params["iq_current_A"],       "A"),
        ("Duty cycle",          params["duty_cycle"],         ""),
        ("Speed",               params["speed_erpm"],         "ERPM"),
        ("Input voltage",       params["input_voltage_V"],    "V"),
        ("Status code",         params["status_code"],        ""),
        ("Position",            params["position_rad"],       "rad"),
        ("Motor ID",            params["motor_id"],           ""),
        ("Vd voltage",          params["vd_V"],               "V"),
        ("Vq voltage",          params["vq_V"],               "V"),
        ("Control mode",        params["ctrl_mode"],          ""),
        ("Encoder angle",       params["encoder_angle_deg"],  "deg"),
        ("Outer encoder angle", params["outer_encoder_deg"],  "deg"),
    ]
    for name, val, unit in rows:
        if isinstance(val, float):
            print(f"  {name:<25s}  {val:>12.4f}  {unit}")
        else:
            print(f"  {name:<25s}  {val:>12}  {unit}")

    # Status code annotation
    sc = params["status_code"]
    status_str = _ERROR_CODES.get(sc, f"Unknown ({sc})")
    print()
    print(f"  Status: {status_str}")

    # ------------------------------------------------------------------
    # 2. COMM_GET_VALUES_SETUP -- selective: MOS temp + voltage + position
    # ------------------------------------------------------------------
    print_section("COMM_GET_VALUES_SETUP (0x10) -- Selected params")

    mask = BITMASK_MOS_TEMP | BITMASK_VOLTAGE | BITMASK_SPEED | BITMASK_POSITION
    cmd2 = build_get_values_setup(mask)
    print(f"  TX: {cmd2.hex().upper()}  (mask=0x{mask:08X})")
    ser.write(cmd2)

    payload2 = recv_frame(ser, timeout=TIMEOUT)
    if payload2 is None:
        print("  No response (motor may not support selective reads in this firmware).")
    else:
        print(f"  RX: {payload2.hex().upper()}")
        # Parse the bitmask reply fields in order: MOS temp, speed, voltage, position
        # (only the bits we set, in LSB-first order)
        d = payload2[1:]
        try:
            idx = 0
            mos    = struct.unpack_from(">h", d, idx)[0] / 10.0;  idx += 2
            spd    = struct.unpack_from(">i", d, idx)[0];          idx += 4
            volt   = struct.unpack_from(">h", d, idx)[0] / 10.0;  idx += 2
            pos    = struct.unpack_from(">f", d, idx)[0];          idx += 4
            print(f"  MOS temp     : {mos:.1f} °C")
            print(f"  Speed        : {spd} ERPM")
            print(f"  Input voltage: {volt:.1f} V")
            print(f"  Position     : {pos:.4f} rad")
        except struct.error:
            print(f"  (raw data too short to parse selected fields)")

    ser.close()
    print()
    print("DONE")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="CubeMars AK60-6 V3.0 serial parameter reader"
    )
    parser.add_argument(
        "--port", default=PORT,
        help=f"Serial port (default: {PORT})"
    )
    args = parser.parse_args()
    run(args.port)


if __name__ == "__main__":
    main()
