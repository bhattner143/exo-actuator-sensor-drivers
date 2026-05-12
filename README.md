# Damiao DM-J4310-2EC Motor Control

Python driver and test scripts for the Damiao DM-J4310-2EC V1.1 geared servo motor.

## Checking the USB-to-CAN Adapter (Linux)

### 1. List connected serial devices
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```
Expected output when the HDSC USB-to-CAN adapter is connected:
```
/dev/ttyACM0
```

### 2. List all USB devices
```bash
lsusb
```
Look for **HDSC CDC Device** (ID `2e88:4603`). That's the Damiao USB-to-CAN adapter.

### 3. Identify the device on a specific port
```bash
udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL"
```
Expected output for the Damiao HDSC adapter:
```
E: ID_VENDOR=HDSC
E: ID_MODEL=CDC_Device
E: ID_VENDOR_ID=2e88
E: ID_MODEL_ID=4603
E: ID_SERIAL=HDSC_CDC_Device_00000000050C
```

The Damiao USB-to-CAN adapter uses the **HDSC CDC** chip (USB-ID `2e88:4603`) and appears as `/dev/ttyACM0`.

---

## Running on Linux vs Windows

`DM_RSIS.py` was written on Windows. Before running on Linux, update the port constants at the top of the file:

```python
# Windows
MOTOR_PORT   = 'COM4'
ARDUINO_PORT = 'COM10'
SAVE_DIR     = r"D:\..."

# Linux equivalent
MOTOR_PORT   = '/dev/ttyACM0'
ARDUINO_PORT = '/dev/ttyUSB0'   # adjust as needed
SAVE_DIR     = '/home/dips/data/RSIS'
```

## Quick-start (motor only)

```bash
python3 DM_Motor_Test.py
```

See `docs/notes/dm_j4310_notes.tex` (or its compiled PDF) for a full protocol reference and audit of the control commands.
