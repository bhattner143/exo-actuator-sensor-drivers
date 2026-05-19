# gs_usb Installation Guide — Jetson Orin Nano (Tegra 5.15.x)

The NVIDIA Tegra kernel does not ship the `gs_usb` driver.  This guide covers
building and installing it as an out-of-tree kernel module so that the
**DSDTech SH-C30A** (and compatible CANable / candleLight) USB-to-CAN adapters
appear as a SocketCAN interface (`can1`).

---

## Hardware

| Item | Detail |
|------|--------|
| Adapter | DSDTech SH-C30A USB-CAN (or any CANable / candleLight clone) |
| USB VID:PID | `1d50:606f` (Geschwister Schneider / OpenMoko) |
| Kernel driver | `gs_usb` |
| Interface created | `can1` (next free `canX` after the native `can0`) |

---

## Prerequisites

| Package | Purpose | Install command |
|---------|---------|-----------------|
| `nvidia-l4t-kernel-headers` | Kernel build tree | pre-installed on JetPack |
| `build-essential` | `gcc`, `make` | `sudo apt-get install build-essential` |
| `wget` | Source download | `sudo apt-get install wget` |
| `can-utils` | `candump`, `cansend` for testing | `sudo apt-get install can-utils` |

Verify the kernel build tree exists before running the script:

```bash
ls /lib/modules/$(uname -r)/build
```

---

## One-time Installation

```bash
cd /home/dips-jetson2/Documents/exo-actuator-sensor-drivers
chmod +x install_gs_usb.sh
sudo ./install_gs_usb.sh
```

The script performs the following steps automatically:

1. **Downloads** `gs_usb.c` from the Linux `v5.15` tag on GitHub (matches
   the Tegra 5.15.x kernel base).
2. **Compiles** `gs_usb.ko` out-of-tree against
   `/lib/modules/$(uname -r)/build`.
3. **Installs** the `.ko` to `/lib/modules/$(uname -r)/extra/gs_usb.ko`
   and runs `depmod -a`.
4. **Auto-load on boot** — creates
   `/etc/modules-load.d/gs_usb.conf`.
5. **udev rule** — creates
   `/etc/udev/rules.d/99-gs-usb-can.rules` which automatically brings the
   adapter up at 1 Mbps whenever it is plugged in.
6. **Loads the module immediately** via `modprobe gs_usb` (no reboot needed).

### Expected output (success)

```
[INFO]  Kernel version : 5.15.185-tegra
[INFO]  Downloading gs_usb.c (Linux v5.15) …
[OK]    Downloaded gs_usb.c
[INFO]  Compiling gs_usb.ko …
  CC [M]  gs_usb.o
  MODPOST Module.symvers
  LD [M]  gs_usb.ko
[OK]    Build succeeded
[OK]    Module installed and depmod updated.
[OK]    Created /etc/modules-load.d/gs_usb.conf
[OK]    udev rule installed: adapter auto-brings-up at 1 Mbps when plugged in.
[OK]    gs_usb loaded.
[OK]    CAN interfaces detected: can0  can1
[OK]    DSDTech SH-C30A USB device recognised (1d50:606f).
```

---

## After Installation

### Verify the adapter is detected

```bash
lsusb | grep 1d50
# Bus 001 Device 005: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter

ip link show can1
# 9: can1: <NOARP,ECHO> mtu 16 qdisc noop state DOWN ...
#     gs_usb: ...
```

### Bring the interface up manually (if udev did not fire)

```bash
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can1   # should show "can state ERROR-ACTIVE"
```

### Run the adapter test suite

```bash
bash test_can_adapter_dstech.sh can1
```

### Use with python-can (SocketCAN backend)

```python
import can

bus = can.interface.Bus(channel="can1", bustype="socketcan")
msg = can.Message(arbitration_id=0x68, data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC], is_extended_id=False)
bus.send(msg)
bus.shutdown()
```

### Use with the CubeMars AK-series V3.0 driver

```bash
# src/ak_v3_can.py reads CAN_CHANNEL from the environment (default "can0").
# Override it to use the DSDTech adapter:
CAN_CHANNEL=can1 python3 tests/cubemars/00_probe_motor.py
```

---

## Installed Files

| Path | Purpose |
|------|---------|
| `/lib/modules/<kver>/extra/gs_usb.ko` | Kernel module |
| `/etc/modules-load.d/gs_usb.conf` | Boot auto-load |
| `/etc/udev/rules.d/99-gs-usb-can.rules` | udev: auto bring-up at 1 Mbps |

---

## Updating After a Kernel Upgrade

The `.ko` file is tied to the exact kernel version.  Re-run the install
script after every `apt upgrade` that updates the kernel:

```bash
sudo ./install_gs_usb.sh
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Module gs_usb not found` after reboot | `/lib/modules/<kver>/extra/` missing for new kernel | Re-run `install_gs_usb.sh` |
| `can1` not created after plug-in | udev rule not loaded | `sudo udevadm control --reload-rules` then re-plug |
| `write: Network is down` | Interface not up | `sudo ip link set can1 up type can bitrate 1000000` |
| `ERROR-PASSIVE` CAN state | No ACK from bus (no motor connected) | Normal when bus has no other node; connect the motor |
| `ERROR-ACTIVE` → `BUS-OFF` | Wiring fault or wrong bitrate | Check CAN-H/CAN-L wiring; confirm motor bitrate is 1 Mbps |
| Build fails: `No such file or directory: Makefile` | `make` run from wrong directory | Always invoke via `sudo ./install_gs_usb.sh`; do not run `make` directly |
| compiler version warning | Minor GCC patch mismatch | Non-fatal; module is produced correctly |
