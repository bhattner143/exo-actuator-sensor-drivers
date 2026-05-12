# Exo Actuator & Sensor Drivers

Python drivers and test scripts for three hardware components on a Jetson Orin Nano bench:

| Hardware | Interface | Driver |
|---|---|---|
| **Damiao DM-J4310-2EC V1.1** | CAN @ 1 Mbps via HDSC USB-to-CAN (`/dev/ttyACM0`) | `src/damiao_bus.py` |
| **CubeMars AK80-9 KV60** | CAN @ 1 Mbps via same adapter | `src/cubemars_bus.py` |
| **AS5048A magnetic encoder** | SPI0 (`/dev/spidev0.0`) | `src/as5048a.py` |

---

## Installation

### 1. Prerequisites

- **Jetson Orin Nano** running JetPack 6 (or any Ubuntu 22.04 host for development)
- [Miniconda / Anaconda](https://docs.conda.io/en/latest/miniconda.html) installed
- HDSC USB-to-CAN adapter plugged in (`/dev/ttyACM0`)
- SPI device-tree overlay installed (see `jetson-orin-spi-overlay-guide/`)

### 2. Create the conda environment

```bash
conda create -n actuators python=3.10 -y
conda activate actuators
```

### 3. Install Python dependencies

```bash
# Core dependencies (all three drivers)
pip install numpy pyserial

# AS5048A encoder only — must be installed system-wide for root (spidev requires root on Jetson)
sudo pip3 install spidev
```

> **Note:** `spidev` is **not** installed into the conda env because encoder scripts must run as
> `sudo python3`. All other scripts run as a normal user inside the conda env.

### 4. Set the `PYTHONPATH` so test scripts find `src/`

```bash
# Add to your shell RC (e.g. ~/.bashrc) or run before each session:
export PYTHONPATH="/home/<user>/Documents/exo-actuator-sensor-drivers/src:$PYTHONPATH"
```

Or create a `.pth` file in the conda env (one-time, no export needed):

```bash
echo "/home/<user>/Documents/exo-actuator-sensor-drivers/src" \
  >> $(python -c "import site; print(site.getsitepackages()[0])")/actuators.pth
```

### 5. (Jetson only) Allow non-root access to the USB-to-CAN adapter

```bash
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect.
```

---

## Verifying the setup

### Damiao motor

```bash
conda activate actuators
cd tests/damiao
python 01_read_params.py        # dumps all firmware parameters over UART
```

### CubeMars motor

```bash
conda activate actuators
cd tests/cubemars
python 01_check_connection.py   # polls 5 CAN feedback frames
```

### AS5048A encoder

```bash
cd tests/encoder
sudo python3 00_spi_connectivity.py   # SPI loopback + chip probe (run first)
sudo python3 01_read_angle.py         # 50 Hz continuous angle readout
```

---

## Checking the USB-to-CAN adapter (Linux)

```bash
ls /dev/ttyACM* /dev/ttyUSB*          # should show /dev/ttyACM0
lsusb | grep HDSC                     # HDSC CDC Device  2e88:4603
udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL"
```

---

## Reference documentation

| Document | Location |
|---|---|
| Damiao protocol notes & audit | `docs/notes/dm_j4310_notes.pdf` |
| AS5048A encoder notes & audit | `docs/notes/as5048a_notes.pdf` |
| Damiao DM-J4310 datasheet | `docs/datasheets/damaio/DM-J4310-en.pdf` |
| CubeMars AK-series datasheet | `docs/datasheets/cubemars/CubeMars-AK-Series.pdf` |
| AS5048A datasheet | `docs/datasheets/encoder/Encoder-AS5048_DS000298_4-00.pdf` |
