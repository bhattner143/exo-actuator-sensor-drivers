"""02 - Write a single motor parameter (with confirmation).

Edit the PARAM and VALUE constants below, then run.
Use save_motor_param() at the end to persist to flash.

WARNING: writing the wrong value (e.g. CAN_ID, NPP, OV) can brick the
motor or require re-calibration. Read first, write second.
"""
import sys
import os
# Add src/ to sys.path so _common.py (which then imports DM_CAN.py) is found
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_motor, safe_disable_close, DM_variable

# === EDIT THESE TWO LINES BEFORE RUNNING ==================================
# PARAM : which register to modify (see DM_variable enum in DM_CAN.py)
# VALUE : new value to write
# Most common use: set MST_ID to 0x11 so the library receives feedback.
PARAM   = DM_variable.MST_ID   # which parameter to write
VALUE   = 0x11                 # new value
# PERSIST = True writes VALUE to flash so it survives power cycle.
# False writes to RAM only -- useful for testing before committing.
PERSIST = False                # if True, save to flash (irreversible without re-write)
# ===========================================================================

# sync_limits=False: no need to sync limits just to change a parameter
motor, mc, ser = open_motor(sync_limits=False)
try:
    # Read back the current value first so the user can see what they are changing
    old = mc.read_motor_param(motor, PARAM)
    print(f"Current {PARAM.name} = {old}")

    # Ask for explicit confirmation -- writing wrong values can break the motor
    ans = input(f"Write {PARAM.name} <- {VALUE}? [y/N] ").strip().lower()
    if ans != "y":
        print("Aborted.")
    else:
        # change_motor_param() writes and reads back; returns True on success
        ok = mc.change_motor_param(motor, PARAM, VALUE)
        print("change_motor_param returned:", ok)
        new = mc.read_motor_param(motor, PARAM)
        print(f"Now {PARAM.name} = {new}")
        if PERSIST:
            # Save ALL parameters to flash.  Disables the motor first (firmware
            # requirement).  The motor must be re-enabled after this call.
            mc.save_motor_param(motor)
            print("Saved to flash.")
finally:
    safe_disable_close(motor, mc, ser)
