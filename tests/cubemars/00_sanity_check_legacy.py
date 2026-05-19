"""Quick sanity check: is the motor physically responding at all?

Sends a legacy MIT frame (standard 11-bit CAN ID, position-first byte order)
to CAN ID 0x02. If we get ANY response, the motor/adapter/wiring is working.

This helps distinguish:
  - Hardware problem (no response at all)
  - Protocol problem (HDSC can't do extended frames)
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from cubemars_bus import CubemarsMotorsBus
from motor_config import CubeMarsBusConfig, CubeMarsMotorConfig

config = CubeMarsBusConfig(
    port="/dev/ttyACM0",
    motors={"test": CubeMarsMotorConfig(can_id=0x02, model="AK60-6")},
)

print("Sending legacy MIT zero-ping to CAN ID 0x02 (standard 11-bit frame)...")
print("If the motor is V3.0-only, it won't respond. If it's dual-mode, it will.")
print()

try:
    with CubemarsMotorsBus(config) as bus:
        # Send a null MIT command (all zeros)
        bus.write("mit_command", {"test": 0.0}, kp=0, kd=0, dq_des=0, tau_ff=0)
        time.sleep(0.2)
        
        # Check if we got feedback
        state = bus.read_state()
        q, dq, tau = state["test"]
        
        if q != 0.0 or dq != 0.0 or tau != 0.0:
            print(f"*** MOTOR RESPONDED (legacy MIT mode) ***")
            print(f"  Position: {q:+.3f} rad")
            print(f"  Velocity: {dq:+.3f} rad/s")
            print(f"  Torque:   {tau:+.3f} N·m")
            print()
            print("This motor accepts legacy MIT (standard 11-bit frames).")
            print("For V3.0 protocol, you'll need a SocketCAN adapter (Canable).")
        else:
            print("No response. Possible causes:")
            print("  1. Motor not powered (18-52 V)")
            print("  2. CAN H/L wires not connected to HDSC adapter")
            print("  3. Motor is in Servo mode (not MIT mode)")
            print("  4. Wrong CAN ID (motor is not 0x02)")
except Exception as e:
    print(f"Error: {e}")
