"""04 - Zero the current position then track rotation relative to that zero.

Steps:
  1. Reads the current angle and latches it as zero (RAM only -- volatile).
  2. Enters a 50 Hz print loop showing:
       - raw 14-bit count (0..16383)
       - angle relative to the zeroed position in degrees (-180..+180)
       - same in radians
       - rotation direction (CW / CCW) compared to previous sample

Press Ctrl+C to stop.  The zero is lost on power cycle (not burned to OTP).

Wiring (Jetson Orin Nano 40-pin header):
    Encoder GND   -> pin 25  (GND)
    Encoder 3V3   -> pin 17  (3.3V)
    Encoder MOSI  -> pin 19  (SPI0_MOSI)
    Encoder MISO  -> pin 21  (SPI0_MISO)
    Encoder SCK   -> pin 23  (SPI0_SCK)
    Encoder CSN   -> pin 24  (SPI0_CS0)
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from encoder_config import DEFAULT_ENCODER_CONFIG
from as5048a import AS5048A

_COUNTS = 16384          # 2^14 counts per revolution
_HALF   = _COUNTS // 2   # 8192 -- wrap threshold for signed delta

PERIOD = 1.0 / 50.0      # 50 Hz sample rate


def signed_delta(a: int, b: int) -> int:
    """Shortest signed path from raw count ``a`` to raw count ``b``.

    Returns a value in (-8192, +8192].  Positive = CCW (increasing count),
    negative = CW (decreasing count) -- assuming the chip counts up CCW.
    """
    d = (b - a) % _COUNTS
    if d > _HALF:
        d -= _COUNTS
    return d


with AS5048A(DEFAULT_ENCODER_CONFIG) as enc:
    print(f"Connected to /dev/spidev{enc.config.bus}.{enc.config.device} "
          f"@ {enc.config.max_hz / 1e6:.1f} MHz, mode {enc.config.mode}")

    # --- Step 1: capture and display current angle, then zero it ----------
    raw_before = enc.read_angle_raw()
    print(f"\nCurrent angle : {raw_before:5d} counts  "
          f"({raw_before * 360.0 / _COUNTS:7.3f} deg)")
    latched = enc.set_zero(burn_otp=False)
    time.sleep(0.05)   # let the chip latch the new offset
    raw_after = enc.read_angle_raw()
    print(f"Zero latched  : {latched:5d} counts  "
          f"({latched * 360.0 / _COUNTS:7.3f} deg offset)")
    print(f"Angle now     : {raw_after:5d} counts  "
          f"({raw_after * 360.0 / _COUNTS:7.3f} deg)  (should be ~0)\n")

    # --- Step 2: track rotation -------------------------------------------
    print("Rotate the magnet.  Ctrl+C to stop.\n")
    print(f"{'raw':>6}  {'deg':>9}  {'rad':>8}  {'dir':>4}  {'total_deg':>10}")
    print("-" * 50)

    prev_raw    = enc.read_angle_raw()
    total_deg   = 0.0   # accumulated signed rotation in degrees

    try:
        while True:
            raw = enc.read_angle_raw()

            # Signed shortest-path delta since last sample
            delta_counts = signed_delta(prev_raw, raw)
            delta_deg    = delta_counts * 360.0 / _COUNTS
            total_deg   += delta_deg

            # Angle relative to zero, wrapped to (-180, +180]
            rel_deg = raw * 360.0 / _COUNTS
            if rel_deg > 180.0:
                rel_deg -= 360.0
            rel_rad = rel_deg * 3.141592653589793 / 180.0

            # Direction based on delta
            if delta_counts > 0:
                direction = " CCW"
            elif delta_counts < 0:
                direction = "  CW"
            else:
                direction = "    "

            print(f"\r{raw:6d}  {rel_deg:9.3f}  {rel_rad:8.4f}  "
                  f"{direction}  {total_deg:10.3f}",
                  end="", flush=True)

            prev_raw = raw
            time.sleep(PERIOD)

    except KeyboardInterrupt:
        print(f"\n\nStopped.  Total rotation: {total_deg:.3f} deg "
              f"({total_deg / 360.0:.3f} turns)")
