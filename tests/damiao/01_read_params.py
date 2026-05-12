"""01 - Read motor parameters.

Connects, then prints firmware/CAN/limit parameters from the motor.
Read-only: no parameter is modified, the motor stays disabled.
"""
import sys
import os
# Add src/ to sys.path so _common.py (which then imports DM_CAN.py) is found
# without any package installation: tests/ -> ../src/
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src'))
from _common import open_motor, safe_disable_close, DM_variable

motor, mc, ser = open_motor(sync_limits=False)
try:
    rows = [
        ("sub_ver",  DM_variable.sub_ver),
        ("hw_ver",   DM_variable.hw_ver),
        ("sw_ver",   DM_variable.sw_ver),
        ("ESC_ID",   DM_variable.ESC_ID),
        ("MST_ID",   DM_variable.MST_ID),
        ("CTRL_MODE",DM_variable.CTRL_MODE),
        ("PMAX",     DM_variable.PMAX),
        ("VMAX",     DM_variable.VMAX),
        ("TMAX",     DM_variable.TMAX),
        ("Gr",       DM_variable.Gr),
        ("ACC",      DM_variable.ACC),
        ("DEC",      DM_variable.DEC),
        ("MAX_SPD",  DM_variable.MAX_SPD),
        ("UV_Value", DM_variable.UV_Value),
        ("OV_Value", DM_variable.OV_Value),
        ("OT_Value", DM_variable.OT_Value),
        ("OC_Value", DM_variable.OC_Value),
        ("Damp",     DM_variable.Damp),
        ("Inertia",  DM_variable.Inertia),
        ("KP_APR",   DM_variable.KP_APR),
        ("KI_APR",   DM_variable.KI_APR),
        ("KP_ASR",   DM_variable.KP_ASR),
        ("KI_ASR",   DM_variable.KI_ASR),
        ("TIMEOUT",  DM_variable.TIMEOUT),
    ]
    print("Motor parameter dump")
    print("-" * 40)
    for name, var in rows:
        val = mc.read_motor_param(motor, var)
        print(f"  {name:10s} = {val}")
finally:
    safe_disable_close(motor, mc, ser)
