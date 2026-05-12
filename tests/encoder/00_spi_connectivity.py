"""00_spi_connectivity.py -- Wiring / SPI connectivity checker (no magnet needed).

A live AS5048A with correct wiring should respond EVEN WITHOUT a magnet.
Expected without a magnet:
    OCF      = 1   (offset compensation finishes regardless of field)
    COMP_HI  = 1   (no magnetic field -> gain cranked up)
    AGC      = 255 (maximum gain, no signal to amplify)
    angle    = garbage (not reliable without field), but NOT always 0x0000

If everything returns 0x0000 the chip is NOT responding -- this is a
wiring/power problem, not a magnet problem.

Run each stage in order and stop when one fails.
"""
import sys
import os
import time
import struct

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src'))

try:
    import spidev
except ImportError:
    print("FAIL: spidev not installed.  Run: pip3 install --user spidev")
    sys.exit(1)

# ---- Stage 0: list available spidev devices ----------------------------
print("=" * 60)
print("Stage 0: available SPI devices")
import glob
devs = sorted(glob.glob("/dev/spidev*"))
print("  Found:", devs if devs else "NONE -- SPI not enabled in jetson-io")

# ---- Stage 1: basic SPI loopback (short MISO pin 21 to MOSI pin 19) ---
print("\nStage 1: SPI loopback test")
print("  Temporarily short pin 19 (MOSI) <-> pin 21 (MISO) with a jumper")
print("  to verify the SPI controller itself is working.")
print("  Disconnect the encoder for this test (it would drive MISO).")

# Auto-skip when stdin is not a TTY (piped/redirected); otherwise prompt.
do_loopback = True
if sys.stdin.isatty():
    try:
        ans = input("  Run loopback now? [Y/n] ").strip().lower()
        do_loopback = ans in ("", "y", "yes")
    except (KeyboardInterrupt, EOFError):
        do_loopback = False
        print("\n  Skipped.")
else:
    print("  (non-interactive run -- running loopback automatically)")

if do_loopback:
    test_pattern = [0xA5, 0x5A, 0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF]
    any_pass = False
    for bus, device in [(1, 0), (0, 0)]:
        node = f"/dev/spidev{bus}.{device}"
        if not os.path.exists(node):
            print(f"  {node:18s} -- skip (node missing)")
            continue
        try:
            spi = spidev.SpiDev()
            spi.open(bus, device)
            spi.max_speed_hz = 500_000
            spi.mode = 1
            rx = spi.xfer2(list(test_pattern))
            spi.close()
        except Exception as e:
            print(f"  {node:18s} -- ERROR: {e}")
            continue
        tx_hex = " ".join(f"{b:02X}" for b in test_pattern)
        rx_hex = " ".join(f"{b:02X}" for b in rx)
        if rx == test_pattern:
            print(f"  {node:18s} -- PASS  tx={tx_hex}  rx={rx_hex}")
            any_pass = True
        elif all(b == 0x00 for b in rx):
            print(f"  {node:18s} -- FAIL  rx all 0x00 (MISO floating; jumper not connected?)")
        elif all(b == 0xFF for b in rx):
            print(f"  {node:18s} -- FAIL  rx all 0xFF (MISO pulled high; check wiring)")
        else:
            # A mismatch where rx bytes partially overlap tx is the normal result
            # when the encoder is connected -- it drives MISO with its own response
            # rather than echoing MOSI.  Stage 2 will confirm whether the chip is live.
            print(f"  {node:18s} -- MISMATCH  tx={tx_hex}  rx={rx_hex}")
            print(f"  {'':18s}    (if encoder is connected, MISO is driven by the chip -- expected)")
    if any_pass:
        print("  -> SPI controller OK.  Remove the jumper and reconnect the encoder.")
    else:
        print("  -> Loopback did not echo cleanly (PASS).")
        print("     If the encoder is connected it will drive MISO and cause a mismatch -- that")
        print("     is expected.  Check Stage 2 results below to confirm chip response.")
        print("     To do a clean loopback: disconnect the encoder, jumper pin 19 to pin 21,")
        print("     then re-run this script.")

# ---- Stage 2: raw SPI probe at different speeds ------------------------
print("\nStage 2: raw probe -- encoder must be wired and powered")
print("  Probing every available /dev/spidevN.0 at mode 1, 100k/500k/1M Hz...")
print("  (A live AS5048A returns NON-ZERO data even without a magnet.)")

candidate_buses = []
for bus in (0, 1):
    if os.path.exists(f"/dev/spidev{bus}.0"):
        candidate_buses.append(bus)

responses = {}  # (bus, hz) -> word
for bus in candidate_buses:
    for hz in (100_000, 500_000, 1_000_000):
        spi = spidev.SpiDev()
        spi.open(bus, 0)
        spi.max_speed_hz = hz
        spi.mode = 1
        # READ 0x3FFD (DIAG+AGC): cmd = 0x7FFD (even parity already).
        spi.xfer2([0x7F, 0xFD])           # send READ 0x3FFD
        rx = spi.xfer2([0xFF, 0xFF])      # NOP, collect response
        word = (rx[0] << 8) | rx[1]
        spi.close()
        responses[(bus, hz)] = word

live_buses = set()
for (bus, hz), word in responses.items():
    if word not in (0x0000, 0xFFFF):
        status = "LIVE"
        live_buses.add(bus)
    elif word == 0xFFFF:
        status = "FLOATING (0xFFFF -- MISO pulled high, no chip)"
    else:
        status = "DEAD (0x0000 -- MISO low, no chip / wrong bus)"
    print(f"  /dev/spidev{bus}.0 @ {hz//1000:4d} kHz -> 0x{word:04X}  {status}")

if not live_buses:
    print("\n  FAIL: no encoder response on any SPI bus.  Check:")
    print("  1. Power:    pin 17 = 3.3V, pin 25 = GND  (multimeter)")
    print("  2. MOSI:     encoder MOSI -> pin 19")
    print("  3. MISO:     encoder MISO -> pin 21")
    print("  4. SCK:      encoder SCK  -> pin 23")
    print("  5. CSN:      encoder CSN  -> pin 24")
    print("  6. Loopback (Stage 1) told you which /dev/spidev maps to the header.")
    print("     If Stage 1 passed on a bus and Stage 2 fails on it -> wiring/power.")
    sys.exit(1)

# Pick a live bus -- prefer the lowest-numbered one
bus_used = sorted(live_buses)[0]
print(f"\n  PASS: encoder responds on /dev/spidev{bus_used}.0")
if bus_used != 0:
    print(f"  NOTE: encoder is on /dev/spidev{bus_used}.0.")
    print(f"        Ensure AS5048AConfig(bus={bus_used}, device=0) in encoder_config.py.")

# ---- Stage 3: parse diagnostics register -------------------------------
print(f"\nStage 3: parse diagnostics register (0x3FFD) on /dev/spidev{bus_used}.0")
spi = spidev.SpiDev()
spi.open(bus_used, 0)
spi.max_speed_hz = 1_000_000
spi.mode = 1

# Clear any latched error first
spi.xfer2([0x40, 0x01])   # READ 0x0001 (clear error flag command)
spi.xfer2([0xFF, 0xFF])   # flush: capture error register response
# Give the chip time to finish internal offset compensation.
# AS5048A datasheet: OCF is set ~10 ms after VDD stable; we wait 100 ms
# to be safe, especially if the chip powered on recently.
time.sleep(0.1)

results = []
for _ in range(8):
    spi.xfer2([0x7F, 0xFD])              # READ 0x3FFD
    rx = spi.xfer2([0xFF, 0xFF])         # collect response (one-frame latency)
    w = (rx[0] << 8) | rx[1]
    data = w & 0x3FFF
    results.append(data)
    time.sleep(0.005)

# If OCF is still 0 after the initial wait, poll for up to 500 ms more.
if not (results[-1] & (1 << 8)):
    for _ in range(10):
        time.sleep(0.05)
        spi.xfer2([0x7F, 0xFD])
        rx = spi.xfer2([0xFF, 0xFF])
        data = (rx[0] << 8 | rx[1]) & 0x3FFF
        results.append(data)
        if data & (1 << 8):   # OCF set -- stop polling
            break

spi.close()

print(f"  Raw diag words: {[hex(r) for r in results[-5:]]}")
for data in results[-3:]:
    agc       = data & 0xFF
    ocf       = bool(data & (1 << 8))
    cof       = bool(data & (1 << 9))
    comp_low  = bool(data & (1 << 10))
    comp_high = bool(data & (1 << 11))
    print(f"  AGC={agc:3d}  OCF={int(ocf)}  COF={int(cof)}  "
          f"COMP_LOW={int(comp_low)}  COMP_HIGH={int(comp_high)}")

print()
# Interpret
last = results[-1]
agc = last & 0xFF
ocf = bool(last & (1 << 8))
comp_high = bool(last & (1 << 11))

if ocf and comp_high and agc > 200:
    print("  RESULT: chip ALIVE, no magnet detected (expected at this stage).")
    print("          Place a diametrically magnetised disc magnet (6-8 mm dia, NdFeB)")
    print("          centred on the shaft above the chip, 0.5-2.5 mm air gap.")
elif ocf and not comp_high:
    print("  RESULT: chip ALIVE and a magnetic field is detected.  Ready to use.")
elif not ocf:
    print("  RESULT: OCF=0 after extended wait -- offset compensation did not finish.")
    print("  Likely causes:")
    print("    a) Power-supply glitch during startup -- power-cycle the encoder and retry.")
    print("    b) SPI mode mismatch -- chip is responding but bits may be phase-shifted.")
    print("       Verify: spi.mode = 1 (CPOL=0, CPHA=1).  Mode 0 gives wrong bit capture.")
    print("    c) Very low supply voltage -- check 3.3 V at encoder VDD with a multimeter.")
