#!/bin/bash
###############################################################################
# install_gs_usb.sh
#
# Builds and installs the gs_usb kernel module for the Jetson Orin Nano
# (Tegra kernel 5.15.x).  Required for the DSDTech SH-C30A / CANable /
# candleLight family of USB-CAN adapters (USB VID:PID 1d50:606f).
#
# The Tegra kernel shipped by NVIDIA does not include gs_usb.  This script
# downloads the matching source from the upstream Linux kernel, compiles it
# as an out-of-tree module against the installed kernel headers, installs the
# .ko file permanently, and configures it to load automatically on boot.
#
# Usage (run once after a fresh Jetpack / kernel install):
#   chmod +x install_gs_usb.sh
#   sudo ./install_gs_usb.sh
#
# After the script completes:
#   - gs_usb is installed in  /lib/modules/$(uname -r)/extra/gs_usb.ko
#   - depmod has been updated, so `modprobe gs_usb` works
#   - /etc/modules-load.d/gs_usb.conf loads it automatically at boot
#   - The adapter appears as can1 (or the next free canX)
#   - Bring it up manually or via udev:
#       sudo ip link set can1 up type can bitrate 1000000
#
# Requirements:
#   - nvidia-l4t-kernel-headers (provides /lib/modules/$(uname -r)/build/)
#   - build-essential (gcc, make)
#   - wget or curl
#   - can-utils   (candump, cansend -- for testing; install if missing)
#
# Re-run after every kernel update (the .ko is kernel-version specific).
###############################################################################

set -euo pipefail

# ── helpers ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${YELLOW}[INFO]  $*${NC}"; }
ok()    { echo -e "${GREEN}[OK]    $*${NC}"; }
err()   { echo -e "${RED}[ERROR] $*${NC}"; exit 1; }

# ── must run as root ──────────────────────────────────────────────────────────
[[ $EUID -eq 0 ]] || err "Run this script with sudo."

KVER="$(uname -r)"
KBUILD="/lib/modules/${KVER}/build"
INSTALL_DIR="/lib/modules/${KVER}/extra"
BUILD_DIR="$(mktemp -d /tmp/gs_usb_build.XXXXXX)"
# Linux tag to download gs_usb.c from.  v5.15 matches the Tegra 5.15.x base.
LINUX_TAG="v5.15"
GS_USB_URL="https://raw.githubusercontent.com/torvalds/linux/${LINUX_TAG}/drivers/net/can/usb/gs_usb.c"

# ── check kernel headers ──────────────────────────────────────────────────────
info "Kernel version : ${KVER}"
info "Build dir      : ${KBUILD}"

[[ -d "${KBUILD}" ]] || \
    err "Kernel build directory not found: ${KBUILD}\n  Install headers first:\n  sudo apt-get install nvidia-l4t-kernel-headers"

# ── check / install build tools ───────────────────────────────────────────────
for tool in gcc make wget; do
    command -v "${tool}" &>/dev/null || {
        info "'${tool}' not found — installing build-essential / wget …"
        apt-get install -y build-essential wget
        break
    }
done

# ── download gs_usb.c ─────────────────────────────────────────────────────────
info "Downloading gs_usb.c (Linux ${LINUX_TAG}) …"
wget -q "${GS_USB_URL}" -O "${BUILD_DIR}/gs_usb.c" || \
    err "Download failed.  Check internet connectivity and the URL:\n  ${GS_USB_URL}"
ok "Downloaded gs_usb.c"

# ── write Makefile ────────────────────────────────────────────────────────────
cat > "${BUILD_DIR}/Makefile" <<'EOF'
obj-m := gs_usb.o

KDIR := /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
EOF

# ── build ─────────────────────────────────────────────────────────────────────
info "Compiling gs_usb.ko …"
make -C "${BUILD_DIR}" 2>&1 | grep -v "^make\[" || true
[[ -f "${BUILD_DIR}/gs_usb.ko" ]] || err "Build failed — gs_usb.ko not produced."
ok "Build succeeded: ${BUILD_DIR}/gs_usb.ko"

# ── install ───────────────────────────────────────────────────────────────────
info "Installing to ${INSTALL_DIR}/gs_usb.ko …"
mkdir -p "${INSTALL_DIR}"
cp "${BUILD_DIR}/gs_usb.ko" "${INSTALL_DIR}/gs_usb.ko"
depmod -a
ok "Module installed and depmod updated."

# ── auto-load on boot ─────────────────────────────────────────────────────────
LOAD_CONF="/etc/modules-load.d/gs_usb.conf"
if [[ ! -f "${LOAD_CONF}" ]]; then
    echo "gs_usb" > "${LOAD_CONF}"
    ok "Created ${LOAD_CONF} — gs_usb will load at every boot."
else
    grep -q "^gs_usb" "${LOAD_CONF}" || echo "gs_usb" >> "${LOAD_CONF}"
    ok "${LOAD_CONF} already exists (gs_usb entry ensured)."
fi

# ── udev rule: bring up canX at 1 Mbps when the adapter is plugged in ─────────
UDEV_RULE="/etc/udev/rules.d/99-gs-usb-can.rules"
if [[ ! -f "${UDEV_RULE}" ]]; then
    cat > "${UDEV_RULE}" <<'UDEV'
# DSDTech SH-C30A / CANable / candleLight (gs_usb): bring up CAN at 1 Mbps
# Triggers whenever a gs_usb network interface appears.
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", \
    ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", \
    RUN+="/sbin/ip link set %k up type can bitrate 1000000"
UDEV
    udevadm control --reload-rules
    ok "udev rule installed: adapter auto-brings-up at 1 Mbps when plugged in."
else
    ok "udev rule already present: ${UDEV_RULE}"
fi

# ── load module now (for immediate use without rebooting) ─────────────────────
if ! lsmod | grep -q "^gs_usb"; then
    info "Loading gs_usb now …"
    modprobe gs_usb && ok "gs_usb loaded." || {
        info "modprobe failed — falling back to insmod …"
        insmod "${INSTALL_DIR}/gs_usb.ko" && ok "gs_usb loaded via insmod."
    }
else
    ok "gs_usb already loaded."
fi

# ── brief adapter check ───────────────────────────────────────────────────────
sleep 1
CAN_IFS=$(ip link show | grep -oP '(?<=\d: )can\d+' || true)
if [[ -n "${CAN_IFS}" ]]; then
    ok "CAN interfaces detected: ${CAN_IFS}"
else
    info "No CAN interfaces found yet. Plug in the adapter if not already done."
fi

if lsusb | grep -q "1d50:606f"; then
    ok "DSDTech SH-C30A USB device recognised (1d50:606f)."
else
    info "DSDTech USB device not seen on USB bus — check the cable."
fi

# ── clean up build dir ────────────────────────────────────────────────────────
rm -rf "${BUILD_DIR}"

echo ""
echo -e "${GREEN}Installation complete.${NC}"
echo ""
echo "Next steps:"
echo "  1. Plug in the DSDTech SH-C30A adapter (if not already plugged in)."
echo "  2. Bring the interface up (udev does this automatically if the rule"
echo "     was just installed; otherwise run manually):"
echo "       sudo ip link set can1 up type can bitrate 1000000"
echo "  3. Test with:"
echo "       bash test_can_adapter_dstech.sh can1"
echo "  4. To use with python-can:"
echo "       import can"
echo "       bus = can.interface.Bus(channel='can1', bustype='socketcan')"
echo ""
echo "Note: Re-run this script after every kernel update."
