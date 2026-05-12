from DM_CAN import *
import serial

Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
serial_device = serial.Serial('/dev/ttyACM0', 921600, timeout=0.5)
MotorControl1=MotorControl(serial_device)
MotorControl1.addMotor(Motor1)

if MotorControl1.switchControlMode(Motor1,Control_Type.POS_VEL):
    print("switch POS_VEL success")

MotorControl1.set_zero_position(Motor1)

while True:
    angle = input("input angle:")
    try:
        angle = float(angle)
        MotorControl1.control_Pos_Vel(Motor1, angle, 1)
    except ValueError:
        break
print("exit")