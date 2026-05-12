"""01 - Continuous angle readout from the AS5048A encoder.

Prints raw counts, degrees, and radians at 50 Hz.  Press Ctrl+C to stop.

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

# Add src/ to sys.path so as5048a / encoder_config are importable.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from encoder_config import DEFAULT_ENCODER_CONFIG
from as5048a import AS5048A

PERIOD = 1.0 / 50.0   # 50 Hz print rate

with AS5048A(DEFAULT_ENCODER_CONFIG) as enc:
    print(f"Connected to /dev/spidev{enc.config.bus}.{enc.config.device} "
          f"@ {enc.config.max_hz/1e6:.1f} MHz, mode {enc.config.mode}")
    print("raw      deg        rad")
    print("-" * 35)
    try:
        while True:
            raw = enc.read_angle_raw()
            deg = raw * (360.0 / 16384.0)
            rad = raw * (6.283185307179586 / 16384.0)
            print(f"\r{raw:5d}   {deg:7.3f}   {rad:7.4f}", end="", flush=True)
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        print("\nStopped.")
