#!/bin/bash
# Install AS5048A Magnetic Encoder Device Tree Overlay for Jetson Orin
#
# Enables SPI1 (Tegra spi@3230000 -> /dev/spidev1.0) on the 40-pin header
# pins 19/21/23/24 so the AS5048A driver in src/as5048a.py can talk to the
# encoder over SPI mode 1.

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  AS5048A Encoder Overlay Installer                          ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}✗ Please run as root: sudo ./install.sh${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Running as root${NC}"

# Check for dtc
if ! command -v dtc &> /dev/null; then
    echo -e "${YELLOW}⚠ Installing device-tree-compiler...${NC}"
    apt-get update
    apt-get install -y device-tree-compiler
fi

echo -e "${GREEN}✓ dtc is available${NC}"

DTS_FILE="jetson-orin-as5048a.dts"
DTBO_FILE="jetson-orin-as5048a.dtbo"
BOOT_LABEL="AS5048A"
BOOT_MENU_LABEL="AS5048A Encoder (SPI1 / spidev1.0)"

# Check for DTS file
if [ ! -f "$DTS_FILE" ]; then
    echo -e "${RED}✗ $DTS_FILE not found${NC}"
    echo -e "${YELLOW}⚠ Please run this script from the directory containing $DTS_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Found $DTS_FILE${NC}"

# Compile
echo -e "${BLUE}ℹ Compiling device tree overlay...${NC}"
dtc -@ -O dtb -o "$DTBO_FILE" "$DTS_FILE"

echo -e "${GREEN}✓ Compiled successfully${NC}"

# Install
echo -e "${BLUE}ℹ Installing to /boot/...${NC}"
cp "$DTBO_FILE" /boot/

echo -e "${GREEN}✓ Installed to /boot/$DTBO_FILE${NC}"

# Detect the correct FDT
echo -e "${BLUE}ℹ Detecting base device tree...${NC}"
FDT_FILE=$(ls /boot/dtb/kernel_tegra*.dtb 2>/dev/null | head -n1)

if [ -z "$FDT_FILE" ]; then
    echo -e "${RED}✗ Could not find base DTB file${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Found base DTB: $FDT_FILE${NC}"

# Configure extlinux.conf
EXTLINUX="/boot/extlinux/extlinux.conf"

if [ ! -f "$EXTLINUX" ]; then
    echo -e "${RED}✗ $EXTLINUX not found${NC}"
    exit 1
fi

# Backup
BACKUP="${EXTLINUX}.backup.$(date +%Y%m%d-%H%M%S)"
echo -e "${BLUE}ℹ Backing up $EXTLINUX to $BACKUP${NC}"
cp "$EXTLINUX" "$BACKUP"

# Remove old entry if it exists
if grep -q "LABEL ${BOOT_LABEL}" "$EXTLINUX"; then
    echo -e "${YELLOW}⚠ ${BOOT_LABEL} boot entry already exists -- removing old entry${NC}"
    sed -i "/^LABEL ${BOOT_LABEL}$/,/^LABEL\|^$/{ /^LABEL ${BOOT_LABEL}$/d; /^LABEL [^A]/Q; d; }" "$EXTLINUX"
fi

# Get the APPEND line from primary entry
echo -e "${BLUE}ℹ Reading primary boot configuration...${NC}"
APPEND_LINE=$(grep "APPEND" "$EXTLINUX" | grep -v "^#" | head -n1 | sed 's/^[[:space:]]*//')

if [ -z "$APPEND_LINE" ]; then
    echo -e "${RED}✗ Could not find APPEND line in primary entry${NC}"
    exit 1
fi

# Create new boot entry
echo -e "${BLUE}ℹ Creating new boot entry...${NC}"

cat >> "$EXTLINUX" << EOF

LABEL ${BOOT_LABEL}
	MENU LABEL ${BOOT_MENU_LABEL}
	LINUX /boot/Image
	FDT $FDT_FILE
	INITRD /boot/initrd
	$APPEND_LINE
	OVERLAYS /boot/$DTBO_FILE
EOF

echo -e "${GREEN}✓ Added ${BOOT_LABEL} boot entry to $EXTLINUX${NC}"

# Set as default
echo -e "${BLUE}ℹ Setting ${BOOT_LABEL} as default boot option...${NC}"
sed -i "s/^DEFAULT .*/DEFAULT ${BOOT_LABEL}/" "$EXTLINUX"

echo -e "${GREEN}✓ Set ${BOOT_LABEL} as default${NC}"

# Summary
echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Installation Complete                                       ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${GREEN}✓ AS5048A overlay installed${NC}"
echo -e "${GREEN}✓ Created new boot entry: '${BOOT_LABEL}'${NC}"
echo -e "${GREEN}✓ Set as default boot option${NC}"
echo ""
echo "Wiring (AS5048A -> Jetson 40-pin header):"
echo "  3V3   -> Pin 17 (3.3V)"
echo "  GND   -> Pin 25 (GND)"
echo "  MOSI  -> Pin 19 (SPI1_MOSI)"
echo "  MISO  -> Pin 21 (SPI1_MISO)"
echo "  SCK   -> Pin 23 (SPI1_SCK)"
echo "  CSN   -> Pin 24 (SPI1_CS0)"
echo ""
echo -e "${YELLOW}⚠ REBOOT REQUIRED${NC}"
echo ""
echo "After reboot, verify with:"
echo "  ls -l /dev/spidev0.0   # header pins map to spi@3210000 on Orin Nano Devkit"
echo "  sudo python3 ../../../tests/encoder/00_spi_connectivity.py"
echo "  sudo python3 ../../../tests/encoder/01_read_angle.py"
echo ""
echo "To revert to original config, edit $EXTLINUX and change:"
echo "  DEFAULT ${BOOT_LABEL}  ->  DEFAULT primary"
echo ""
echo -ne "${YELLOW}Reboot now? [y/N] ${NC}"
read -r response

if [[ "$response" =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}ℹ Rebooting...${NC}"
    reboot
else
    echo -e "${BLUE}ℹ Remember to reboot before testing!${NC}"
fi
