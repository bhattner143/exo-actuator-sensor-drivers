"""11_cable_wind_to_tension.py -- Damiao DM-J4310-2EC: cable slack removal.

PURPOSE
-------
This script is designed for cable-driven robots where a cable is wrapped
around the motor drum but has slack (it is not taut).  The script commands
the motor to wind the cable at a low, constant torque until the cable
becomes taut, then stops.

CONTROL STRATEGY
----------------
MIT Torque mode (Kp=0, Kd=0, tau_ff = WIND_TORQUE_NM).

  - Slack cable   -> motor encounters no resistance -> spins freely.
  - Taut cable    -> cable tension opposes the torque -> motor decelerates
                     and stalls naturally.
  - Stall detect  -> |dq| < V_STALL_EPS for T_STALL_CONFIRM seconds
                     AND tau_est > TAU_TAUT_MIN N.m (cross-check).

STATE MACHINE
-------------
  INIT     : Apply torque, start timer.
  WINDING  : Motor spinning; cable is still slack.
  CONFIRM  : Velocity dropped; waiting for stall confirmation period.
  DONE     : Cable taut; torque zeroed (or drum held via position control).
  TIMEOUT  : Did not reach taut within TIMEOUT_S; torque zeroed; error flag.

HARDWARE SETUP
--------------
  Motor : Damiao DM-J4310-2EC V1.1, CAN1 Mbps, ESC_ID = 0x01, MST_ID = 0x11.
  Host  : Jetson Orin Nano, USB-to-CAN adapter on /dev/ttyACM0.
  Wiring: Ensure cable exit path is clear.  Secure the distal attachment
          point before running.  Keep the cable force path unobstructed.

PARAMETER TUNING
----------------
  Start with WIND_TORQUE_NM = 0.2 N.m.  Increase if the motor does not
  reliably move against cable bending stiffness.  Never exceed the cable's
  rated safe working load divided by the drum radius.

  V_STALL_EPS : velocity below which the motor is considered stalled.
                0.05-0.15 rad/s is typical.  Raise in noisy environments.

  T_STALL_CONFIRM : how long the motor must be stalled to confirm taut.
                    Prevents false triggers from momentary slowdowns.
                    0.25-0.5 s is typical.

  TAU_TAUT_MIN : minimum torque estimate that must be seen during stall.
                 Guards against a mechanical jam (friction stall without
                 cable tension).  Set to ~50% of WIND_TORQUE_NM.

  TIMEOUT_S : abort if cable does not become taut within this time.
              Set to max expected winding time = slack_length / (drum_r * dq).

POST-STALL BEHAVIOUR
--------------------
  POST_STALL_HOLD = True  -> Immediately switch to MIT position hold
                             (low Kp/Kd) to keep the drum at the taut
                             angle.  Use when subsequent control must
                             maintain cable tension.
  POST_STALL_HOLD = False -> Zero torque and let the motor go passive.
                             Use when the distal attachment is spring-
                             tensioned and will hold cable tension.

SAFETY
------
  - Ctrl+C zeroes torque and disables the motor cleanly (finally block).
  - A hard timeout (TIMEOUT_S) prevents continuous rotation if the cable
    is missing or the distal attachment fails.
  - WIND_TORQUE_NM is intentionally low; verify drum radius and cable
    safe working load before raising it.
  - This script was written for single-cable commissioning.  For
    multi-cable robots, run one cable at a time.

RUNNING
-------
  cd /home/<user>/Documents/exo-actuator-sensor-drivers
  python3 tests/damiao/11_cable_wind_to_tension.py

  Or from any directory if the repo src/ is on PYTHONPATH:
  PYTHONPATH=/path/to/repo/src python3 11_cable_wind_to_tension.py
"""

import sys
import os
import time

# ---------------------------------------------------------------------------
# Path setup: allow running from any directory.
# ---------------------------------------------------------------------------
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src"))

from _common import open_bus  # noqa: E402  (import after path setup)

# ===========================================================================
# ---- CONFIGURABLE PARAMETERS ----------------------------------------------
# ===========================================================================

WIND_TORQUE_NM   = 0.3    # N.m  winding torque (positive = CCW output shaft)
                           #      Adjust sign if cable unwinds instead of winds.

V_STALL_EPS      = 0.10   # rad/s  |dq| below this -> motor treated as stalled

T_STALL_CONFIRM  = 0.35   # s  motor must be stalled this long to confirm taut

TAU_TAUT_MIN     = 0.15   # N.m  torque estimate must exceed this during stall
                           #      (guards against friction-only stall without tension)

TIMEOUT_S        = 20.0   # s  abort if cable does not go taut within this time

WINDING_HZ       = 100    # Hz  control loop rate (do not exceed ~200 Hz over CAN)

# Post-stall behaviour
POST_STALL_HOLD  = True   # True  -> hold drum angle with low Kp/Kd after taut
                           # False -> release (zero torque) after taut

KP_HOLD          = 5.0    # position spring for post-stall hold
KD_HOLD          = 0.5    # velocity damping for post-stall hold

# Minimum travel before stall detection is armed.
# Prevents the motor stalling on initial static friction from being
# misinterpreted as cable tension.
MIN_TRAVEL_RAD   = 0.10   # rad  output-shaft travel required before stall armed

# ===========================================================================
# ---- STATE MACHINE --------------------------------------------------------
# ===========================================================================
INIT    = "INIT"
WINDING = "WINDING"
CONFIRM = "CONFIRM"
DONE    = "DONE"
TIMEOUT = "TIMEOUT"

# ===========================================================================
# ---- MAIN -----------------------------------------------------------------
# ===========================================================================


def run_cable_wind_to_tension():
    """Wind the cable drum until the cable goes taut, then stop."""

    dt = 1.0 / WINDING_HZ

    print("=" * 60)
    print("  DM-J4310-2EC Cable Slack Removal")
    print("=" * 60)
    print(f"  Winding torque   : {WIND_TORQUE_NM:+.3f} N.m")
    print(f"  Stall velocity   : |dq| < {V_STALL_EPS:.3f} rad/s")
    print(f"  Stall confirm    : {T_STALL_CONFIRM:.2f} s")
    print(f"  Taut torque min  : {TAU_TAUT_MIN:.3f} N.m")
    print(f"  Timeout          : {TIMEOUT_S:.1f} s")
    print(f"  Post-stall hold  : {POST_STALL_HOLD} "
          f"(Kp={KP_HOLD}, Kd={KD_HOLD})" if POST_STALL_HOLD else "")
    print("-" * 60)

    # -- open_bus enters MIT mode and enables the motor ----------------------
    with open_bus() as bus:
        state          = INIT
        q_start        = None     # shaft angle at start (rad)
        q_stall        = None     # shaft angle when stall confirmed (rad)
        stall_start_t  = None     # time when velocity first dropped (s)
        t_start        = time.monotonic()

        print(f"[cable_wind] Motor enabled.  Starting winding ...")
        print()

        try:
            while True:
                t_now = time.monotonic()
                elapsed = t_now - t_start

                # --- read motor state ----------------------------------------
                state_dict = bus.read_state()
                q, dq, tau = state_dict["j1"]   # output shaft: rad, rad/s, N.m

                # --- record start position -----------------------------------
                if q_start is None:
                    q_start = q

                travel = abs(q - q_start)

                # --- print status every 10 ticks ----------------------------
                tick = int(elapsed * WINDING_HZ)
                if tick % 10 == 0:
                    print(
                        f"[cable_wind] {state:<8s} | "
                        f"t={elapsed:5.1f}s  q={q:+6.3f} rad  "
                        f"dq={dq:+6.3f} rad/s  tau={tau:+5.3f} Nm  "
                        f"travel={travel:.3f} rad"
                    )

                # --- state machine -------------------------------------------

                if state == INIT:
                    # Apply winding torque.
                    bus.write("goal_torque", {"j1": WIND_TORQUE_NM})
                    state = WINDING

                elif state == WINDING:
                    bus.write("goal_torque", {"j1": WIND_TORQUE_NM})

                    # Check timeout.
                    if elapsed > TIMEOUT_S:
                        state = TIMEOUT
                        break

                    # Only arm stall detection after minimum travel.
                    if travel < MIN_TRAVEL_RAD:
                        continue

                    # Velocity dropped -> enter CONFIRM.
                    if abs(dq) < V_STALL_EPS:
                        stall_start_t = t_now
                        state = CONFIRM

                elif state == CONFIRM:
                    bus.write("goal_torque", {"j1": WIND_TORQUE_NM})

                    # Velocity recovered -> false alarm, go back to WINDING.
                    if abs(dq) >= V_STALL_EPS:
                        stall_start_t = None
                        state = WINDING
                        continue

                    stall_duration = t_now - stall_start_t

                    # Stall confirmed for required duration AND torque present.
                    torque_ok = (abs(tau) >= TAU_TAUT_MIN)
                    time_ok   = (stall_duration >= T_STALL_CONFIRM)

                    if time_ok and torque_ok:
                        q_stall = q
                        state = DONE
                        break

                    # Stall confirmed by time alone (torque may be noisy at
                    # very low WIND_TORQUE_NM values -- trust the velocity).
                    if time_ok and (WIND_TORQUE_NM < 2 * TAU_TAUT_MIN):
                        print(
                            "[cable_wind] WARNING: torque estimate below "
                            "TAU_TAUT_MIN -- stopping on velocity alone."
                        )
                        q_stall = q
                        state = DONE
                        break

                    # Check timeout.
                    if elapsed > TIMEOUT_S:
                        state = TIMEOUT
                        break

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n[cable_wind] Ctrl+C -- aborting.")
            state = TIMEOUT

        finally:
            # --- always zero torque on exit ---------------------------------
            try:
                bus.write("goal_torque", {"j1": 0.0})
                time.sleep(0.05)   # one final zero-torque frame
            except Exception:
                pass

        # --- post-loop reporting -------------------------------------------
        if state == DONE:
            wind_time = time.monotonic() - t_start
            print()
            print(f"[cable_wind] DONE: cable taut at q = {q_stall:+.4f} rad "
                  f"after {wind_time:.2f} s  "
                  f"(travel = {abs(q_stall - q_start):.4f} rad).")

            if POST_STALL_HOLD:
                print(f"[cable_wind] Holding drum at q = {q_stall:+.4f} rad "
                      f"with Kp={KP_HOLD}, Kd={KD_HOLD}.")
                print("[cable_wind] Press Ctrl+C to release.")
                try:
                    while True:
                        bus.write(
                            "goal_position",
                            {"j1": q_stall},
                            kp=KP_HOLD,
                            kd=KD_HOLD,
                        )
                        time.sleep(dt)
                except KeyboardInterrupt:
                    print("\n[cable_wind] Released.")
                finally:
                    bus.write("goal_torque", {"j1": 0.0})
                    time.sleep(0.05)

        elif state == TIMEOUT:
            print()
            print(f"[cable_wind] TIMEOUT after {TIMEOUT_S:.1f} s: "
                  "cable did not reach tension.  Check:")
            print("  - Cable is attached to the drum and distal point.")
            print("  - WIND_TORQUE_NM is large enough to overcome friction.")
            print("  - Winding direction is correct (try negating WIND_TORQUE_NM).")
            sys.exit(1)

        else:
            print(f"\n[cable_wind] Ended in unexpected state: {state}")
            sys.exit(2)


if __name__ == "__main__":
    run_cable_wind_to_tension()
