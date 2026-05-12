"""02 - AS5048A diagnostics + magnet placement check.

Prints AGC, CORDIC magnitude and the four diagnostic flags.  Use this
to verify the magnet is at the correct distance/displacement:

    comp_low  = 1  -> field too STRONG  (move magnet away)
    comp_high = 1  -> field too WEAK    (move magnet closer)
    cof       = 1  -> CORDIC overflow   (angle invalid; check magnet)
    ocf       = 1  -> offset compensation finished  (should be 1 after power-up)

Target: AGC ~ 128, both comp flags 0, magnitude in mid-range.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))
from encoder_config import DEFAULT_ENCODER_CONFIG
from as5048a import AS5048A

with AS5048A(DEFAULT_ENCODER_CONFIG) as enc:
    print("AGC   MAG     OCF  COF  COMP_HI  COMP_LO   verdict")
    print("-" * 60)
    try:
        while True:
            diag = enc.read_diagnostics()
            mag  = enc.read_magnitude()
            if diag["comp_low"]:
                verdict = "FIELD TOO STRONG - increase air gap"
            elif diag["comp_high"]:
                verdict = "FIELD TOO WEAK   - decrease air gap"
            elif diag["cof"]:
                verdict = "CORDIC OVERFLOW  - check magnet alignment"
            elif not diag["ocf"]:
                verdict = "offset comp not finished (wait...)"
            else:
                verdict = "OK"
            print(f"\r{diag['agc']:3d}   {mag:5d}    {int(diag['ocf'])}    "
                  f"{int(diag['cof'])}    {int(diag['comp_high'])}        "
                  f"{int(diag['comp_low'])}        {verdict:40s}",
                  end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")
