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

# CubeMars V3.0 with SocketCAN adapter (Canable / Waveshare) — optional
# Only needed if using ak_v3_can.py with a SocketCAN-compatible USB-CAN adapter.
# NOT needed for legacy MIT mode (cubemars_bus.py) or HDSC adapter (ak_v3_serial.py).
pip install python-can

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

---

## Actuator Identification Framework

This repo integrates with the **PyDrake + Isaac Sim** dual-simulator framework (see parent `Isaac_sim_robotics/` repo) for:

1. **Hardware-to-Sim Methodology** (`docs/notes/skyntefic_hardware_to_sim_methodology.tex`):
   - Pulse identification: sharp command response to identify motor inertia & damping
   - Sinusoidal frequency-response tests: encoder-trace fitting to second-order plant model
   - Cross-validation: PyDrake simulation vs. measured hardware traces

2. **Cable-Driven SEA Tuning** (see `Isaac_sim_robotics/.github/skills/sea-tuning/SKILL.md`):
   - Spring stiffness tuning for cable-driven joints
   - Damping coefficient selection for tracking performance
   - Motor bandwidth matching to cable resonance

3. **Exosuit Co-Contraction & Cable Routing** (see `Isaac_sim_robotics/.github/instructions/exosuit-cables.instructions.md`):
   - Dual-groove pulley geometry for antagonistic exo motors
   - Effective stiffness calculation: $K_{\mathrm{eff}} = 2\,k_{\mathrm{exo}}\,r_{\mathrm{exo}}^2$
   - Method A (offset pulleys) vs Method B (centred elbow pulley) implementations

4. **Hardware Experiment Protocols** (see parent `Isaac_sim_robotics/notes_all/` for meeting notes):
   - Kinesthetic teaching with impedance control (low Kp/Kd for back-drivability)
   - Perturbation-response measurements for parameter identification
   - Trajectory recording & replay from encoder traces

---

## Reference documentation

| Document | Location | Purpose |
|---|---|---|
| Damiao protocol notes & audit | `docs/notes/dm_j4310_notes.tex` → PDF | Motor control protocol & firmware audit |
| CubeMars notes & audit | `docs/notes/cubemars_notes.tex` → PDF | AK60-6 V3.0 & AK80-8 V1.x protocol audit |
| AS5048A encoder notes & audit | `docs/notes/as5048a_notes.tex` → PDF | SPI encoder protocol & parity check |
| Hardware-to-Sim Methodology | `docs/notes/skyntefic_hardware_to_sim_methodology.tex` → PDF | Actuator ID via sharp/sine tests, encoder fitting |
| Trajectory Control (CubeMars) | `docs/notes/trajectory_control_cubemars.tex` → PDF | Kinesthetic teaching & replay framework |
| Damiao DM-J4310 datasheet | `docs/datasheets/damaio/DM-J4310-en.pdf` | Motor specs & firmware parameters |
| CubeMars AK-series datasheet | `docs/datasheets/cubemars/CubeMars-AK-Series.pdf` | Extended & standard CAN protocols |
| AS5048A datasheet | `docs/datasheets/encoder/Encoder-AS5048_DS000298_4-00.pdf` | 14-bit absolute encoder specs |
