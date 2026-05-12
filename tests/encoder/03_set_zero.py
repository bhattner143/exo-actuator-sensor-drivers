"""03 - Latch the current shaft angle as the new zero (RAM only).

Volatile: the zero is stored in 0x0016 / 0x0017 but NOT burned to OTP,
so it is lost on power cycle.  To make it permanent, edit this script
and pass ``burn_otp=True`` to ``enc.set_zero()`` -- this can only be
done ONCE per chip, so be sure first.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from encoder_config import DEFAULT_ENCODER_CONFIG
from as5048a import AS5048A

with AS5048A(DEFAULT_ENCODER_CONFIG) as enc:
    print(f"Angle before zeroing : {enc.read_angle_deg():7.3f} deg")
    latched = enc.set_zero(burn_otp=False)
    print(f"Latched offset       : {latched:5d} counts "
          f"({latched * 360.0 / 16384.0:7.3f} deg)")
    time.sleep(0.05)
    print(f"Angle after zeroing  : {enc.read_angle_deg():7.3f} deg "
          "(should be ~0)")
