# Damiao Python Library

This library supports macOS, Linux, and Windows.

**Join the QQ group: 677900232 for Damiao motor technical discussion. Visit the Damiao shop to browse and purchase:** [Home - Damiao Intelligent Control Enterprise Store - Taobao](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX)

### 1. Import the Damiao library

The default folder contains two files. `DM_CAN.py` is the motor library. Use it like this:

```python
from DM_CAN import *
```

The motor library depends on `serial` and `numpy`, so install those packages. **The exact requirements are listed in `requirements.txt`; this is the version I tested. Older versions should generally work too.**

### 2. Define control objects

Create one motor object for each motor. Important: do not set `masterid` to `0x00`.

```python
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x13)
```

The first argument is the motor type. The second is the Slave ID, which is the motor CAN ID. The third argument is the Master ID, which is the host ID. It is recommended that each Master ID be different and typically one value higher than the Slave ID.

For example, if Motor1 has Slave ID `0x01`, set Master ID to `0x11`. This is best.

**Master ID and Slave ID must be configured in the Damiao PC software! If there is a problem, first check that the Master ID does not conflict with the Slave ID and is not `0x00`.**

**Do not set Master ID to `0x00`.**

Python uses the serial port at 921600 baud. Select the serial port for your system. The demo is on Windows and uses `COM8`:

```python
serial_device = serial.Serial('COM8', 921600, timeout=0.5)
```

Initialize the motor control object by passing the serial object:

```python
MotorControl1 = MotorControl(serial_device)
```

### 3. Motor status

#### 3.1 Add motors

Add motors with `addMotor`:

```python
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
```

#### 3.2 Enable motors

**Recommendation: if you want to modify motor parameters, enable motors last.**

```python
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
```

For compatibility with older firmware, the enable call may require specifying the mode. The enabled mode must match the motor's current mode; this does not change the motor's internal mode.

```python
MotorControl1.enable_old(Motor1, Control_Type.MIT)
MotorControl1.enable_old(Motor2, Control_Type.POS_VEL)
MotorControl1.enable_old(Motor3, Control_Type.VEL)
```

#### 3.3 Set zero position

With the motor disabled, move it to the desired zero position and run the following lines. The current position will be set as the motor zero point:

```python
MotorControl1.set_zero_position(Motor3)
MotorControl1.set_zero_position(Motor6)
```

#### 3.4 Disable motors

```python
MotorControl1.disable(Motor3)
MotorControl1.disable(Motor6)
```

#### 3.5 Read motor status

Damiao motors normally require sending a control frame before current torque, position, velocity, and other data are updated. If you want the motor's current status without sending a control command, use:

```python
MotorControl1.refresh_motor_status(Motor1)
print("Motor1:", "POS:", Motor1.getPosition(), "VEL:", Motor1.getVelocity(), "TORQUE:", Motor1.getTorque())
```

The `refresh_motor_status` function retrieves the current motor state and stores it in the corresponding motor object.

### 4. Motor control modes

**Recommendation: add a 1-2 ms delay after each control frame. USB-to-CAN converters usually buffer commands and may work without delay, but a short delay is still recommended.**

#### 4.1 MIT mode

After enabling the motor, you can control it in MIT mode. This mode is recommended.

```python
MotorControl1.controlMIT(Motor1, 50, 0.3, 0, 0, 0)
```

#### 4.2 Position-velocity mode

Position-velocity mode uses the motor object, target position, and target velocity. Detailed parameter documentation is available in the function docs and can be viewed in PyCharm or another IDE.

Example:

```python
q = math.sin(time.time())
MotorControl1.control_Pos_Vel(Motor1, q * 10, 2)
```

#### 4.3 Velocity mode

Example: the first argument is the motor object and the second is the motor velocity.

```python
q = math.sin(time.time())
MotorControl1.control_Vel(Motor1, q * 5)
```

New Damiao firmware currently supports switching modes.

#### 4.4 Force-position hybrid mode

The first argument is the motor object, the second is the target position, the third is the velocity range (0-10000), and the fourth is the current range (0-10000). See the Damiao documentation for full details.

Example:

```python
MotorControl1.control_pos_force(Motor1, 10, 1000, 100)
```

### 5. Read motor state

Each motor object's state values are stored in that object. Use the following functions to read them.

**Note: Damiao motor state is updated only after sending a control frame or calling `refresh_motor_status`.**

**Damiao motors use a send-and-receive pattern: the motor returns its current state only after a command is sent.**

```python
vel = Motor1.getVelocity()    # get motor velocity
pos = Motor1.getPosition()    # get motor position
tau = Motor1.getTorque()      # get motor torque
```

```python
MotorControl1.refresh_motor_status(Motor1)
print("Motor1:", "POS:", Motor1.getPosition(), "VEL:", Motor1.getVelocity(), "TORQUE:", Motor1.getTorque())
```

### 6. Change motor internal parameters

New Damiao firmware supports modifying motor modes and other parameters over CAN. Firmware version 5013 or higher is required. Consult Damiao support for details. **Note: save and modify parameters only while the motor is disabled.**

#### 6.1 Change control mode

Use the following function to modify the motor's control mode. Supported modes are `MIT`, `POS_VEL`, `VEL`, and `Torque_Pos`. The mode change is performed online. The code returns a value; if it is `True`, the setting succeeded. If it is not `True`, it does not necessarily mean the change failed.

```python
if MotorControl1.switchControlMode(Motor1, Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor2, Control_Type.VEL):
    print("switch VEL success")
```

**If you want to keep the motor control mode, save the parameters afterward.**

#### 6.2 Save parameters

By default, motor parameter changes are not saved to flash. Use the following command to save settings to the motor's flash memory. Example: **this command saves all modified parameters for `Motor1` to flash. Perform this while the motor is disabled.** The function automatically disables the motor internally to prevent saving while enabled.

```python
MotorControl1.save_motor_param(Motor1)
```

#### 6.3 Read internal register parameters

Many internal parameters can be read over CAN. See the Damiao manual for the parameter list. Readable parameters are defined in the `DM_variable` enum. Use `read_motor_param` to read them.

```python
print("PMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.PMAX))
```

print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))
print("Motor2:")
print("PMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor2,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.TMAX))
```

并且每次读取参数后，当前的参数也会同时存在对应的电机类里面，通过getParam这个函数进行读取。

```python
print("PMAX",Motor1.getParam(DM_variable.PMAX))
```

#### 6.4改写内部寄存器参数

内部寄存器有一部分是支持修改的，一部分是只读的（无法修改）。通过调用change_motor_param这个函数可以进行寄存器内部值修改。并且也如同上面读寄存器的操作一样，他的寄存器的值也会同步到电机对象的内部值，可以通过Motor1.getParam这个函数进行读取。

**请注意这个修改内部寄存器参数，掉电后会恢复为修改前的，并没有保存**

```python
if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
   print("write success")
```

