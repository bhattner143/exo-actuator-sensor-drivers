import math
from DM_CAN import *
import serial
import time
import csv
import keyboard
import matplotlib.pyplot as plt
from collections import deque
import os
import numpy as np  # 必须安装: pip install numpy

# ================= 配置区域 =================
MOTOR_PORT = 'COM4'
MOTOR_BAUDRATE = 921600

ARDUINO_PORT = 'COM10'
ARDUINO_BAUDRATE = 250000 

SAVE_DIR = r"D:\PhD\paper\RAL\payload test code\damiao\RSIS"

# --- 算法配置 ---
CONFIG = {
    'WINDOW_SIZE': 5,            
    'INIT_DURATION': 10.0,        # 初始化总时长
    'INIT_IGNORE_FIRST': 6.0,    # 忽略前3秒
    'COOLDOWN_DURATION': 1.0,    
    'SLIP_COUNT_THRESHOLD': 4,   
    
    # [新增] IMU 加速度滑移阈值
    'ACC_SLIP_THRESHOLD': 0.8, 

    # 电阻算法阈值 (高灵敏度版)
    'EMA_ALPHA_BASE': 0.20,
    'EMA_ALPHA_MIN': 0.08,
    'EMA_ALPHA_MAX': 0.30,
    'THRESH': {
        'SLOPE_POS_PERC': 98.0,
        'SLOPE_NEG_PERC': 1.5,
        'STD_PERC': 98.0,
        'MARGIN': 1.0,
        'SLOPE_POS_MIN': 0.0004,
        'SLOPE_NEG_MIN': 0.002,
        'STD_MIN': 0.004
    }
}

# ================= 全局算法变量 =================
algo_params = {
    'R0': [None] * 5,             
    'alpha': [0.2] * 5,           
    'ema_state': [None] * 5,      
    'thr_slope_pos': [0.0] * 5,   
    'thr_slope_neg': [0.0] * 5,   
    'thr_std': [0.01] * 5,
    'Acc0': [None] * 3  # X, Y, Z
}

runtime_buffers = {
    'init_buffer': [[] for _ in range(5)],  
    'init_acc_buffer': [[] for _ in range(3)], 
    'slide_window': [deque(maxlen=CONFIG['WINDOW_SIZE']) for _ in range(5)], 
    'slip_counters': [0] * 5      
}

# 状态定义
SENSING_IDLE = 0     
SENSING_INIT = 1     # 初始化/标定阶段
SENSING_MONITOR = 2  # 监测/滑移识别阶段
sensing_state = SENSING_IDLE
sensing_start_time = 0.0 

# ================= 算法函数 =================

def debug_log(msg):
    print(f"[Algo] {msg}")

def reset_algorithm():
    global sensing_state, algo_params, runtime_buffers
    sensing_state = SENSING_IDLE
    algo_params['R0'] = [None] * 5
    algo_params['Acc0'] = [None] * 3 
    algo_params['ema_state'] = [None] * 5
    
    runtime_buffers['init_buffer'] = [[] for _ in range(5)]
    runtime_buffers['init_acc_buffer'] = [[] for _ in range(3)] 
    runtime_buffers['slide_window'] = [deque(maxlen=CONFIG['WINDOW_SIZE']) for _ in range(5)]
    runtime_buffers['slip_counters'] = [0] * 5
    debug_log("Algorithm Reset.")

def ema_filter(val, idx):
    alpha = algo_params['alpha'][idx]
    if algo_params['ema_state'][idx] is None:
        algo_params['ema_state'][idx] = val
    else:
        algo_params['ema_state'][idx] = (1 - alpha) * algo_params['ema_state'][idx] + alpha * val
    return algo_params['ema_state'][idx]

def compute_calibration_params():
    global algo_params
    
    # 1. 计算切除比例
    ignore_ratio = CONFIG['INIT_IGNORE_FIRST'] / CONFIG['INIT_DURATION']
    if ignore_ratio > 0.95: ignore_ratio = 0.95

    # 2. 电阻参数
    res_buffer = runtime_buffers['init_buffer']
    valid_res_data = []
    
    for ch_data in res_buffer:
        if len(ch_data) < 10: 
            valid_res_data.append(np.array(ch_data))
        else:
            cut_idx = int(len(ch_data) * ignore_ratio) 
            valid_res_data.append(np.array(ch_data[cut_idx:]))

    for i in range(5):
        if len(valid_res_data[i]) > 0:
            algo_params['R0'][i] = float(np.mean(valid_res_data[i]))
        else:
            algo_params['R0'][i] = 1.0 

    delta_init = []
    sigmas = []
    for i in range(5):
        if algo_params['R0'][i] == 0: d = np.zeros(1)
        else: d = (valid_res_data[i] - algo_params['R0'][i]) / algo_params['R0'][i]
        delta_init.append(d)
        if d.size > 1:
            med = np.median(d)
            mad = np.median(np.abs(d - med))
            sigma = 1.4826 * mad if mad > 0 else float(np.std(d))
        else:
            sigma = 0.0
        sigmas.append(sigma)
    
    med_sigma = np.median([s for s in sigmas if s > 0]) if any(s > 0 for s in sigmas) else 0.0
    for i, s in enumerate(sigmas):
        if med_sigma == 0: a = CONFIG['EMA_ALPHA_BASE']
        else: 
            ratio = med_sigma / max(s, 1e-12)
            a = CONFIG['EMA_ALPHA_BASE'] * ratio
        algo_params['alpha'][i] = max(CONFIG['EMA_ALPHA_MIN'], min(CONFIG['EMA_ALPHA_MAX'], a))

    th = CONFIG['THRESH']
    W = CONFIG['WINDOW_SIZE']
    for i in range(5):
        a = algo_params['alpha'][i]
        d = delta_init[i]
        sim_ema = []
        x = None
        for v in d:
            x = v if x is None else (1 - a) * x + a * v
            sim_ema.append(x)
        sim_ema = np.array(sim_ema)
        
        slopes, stds = [], []
        if sim_ema.size >= W:
            x_idx = np.arange(W)
            for t in range(W-1, sim_ema.size):
                w_data = sim_ema[t-W+1 : t+1]
                s, _ = np.polyfit(x_idx, w_data, 1)
                slopes.append(s)
                stds.append(np.std(w_data))
        
        def perc(vals, p, default):
            return float(np.percentile(vals, p)) if len(vals) > 0 else float(default)
        pos_vals = [s for s in slopes if s > 0]
        neg_vals = [s for s in slopes if s < 0]
        p_thr = perc(pos_vals, th['SLOPE_POS_PERC'], th['SLOPE_POS_MIN'])
        n_thr = perc(neg_vals, th['SLOPE_NEG_PERC'], -th['SLOPE_NEG_MIN'])
        s_thr = perc(stds, th['STD_PERC'], th['STD_MIN'])
        algo_params['thr_slope_pos'][i] = max(th['SLOPE_POS_MIN'], p_thr * th['MARGIN'])
        algo_params['thr_slope_neg'][i] = min(-th['SLOPE_NEG_MIN'], n_thr * th['MARGIN'])
        algo_params['thr_std'][i] = max(th['STD_MIN'], s_thr * th['MARGIN'])

    # 3. IMU 参数 (Acc0)
    acc_buffer = runtime_buffers['init_acc_buffer']
    for i in range(3):
        ch_data = acc_buffer[i]
        if len(ch_data) < 10:
            algo_params['Acc0'][i] = 0.0
        else:
            cut_idx = int(len(ch_data) * ignore_ratio)
            valid_acc = np.array(ch_data[cut_idx:])
            algo_params['Acc0'][i] = float(np.mean(valid_acc))

    debug_log(f"Calibration Done. R0: {[round(r,1) for r in algo_params['R0']]}")
    debug_log(f"IMU Acc0: {[round(a,3) for a in algo_params['Acc0']]}")

def check_for_slip(current_vals_filtered):
    flags = [0] * 5
    for i in range(5):
        runtime_buffers['slide_window'][i].append(current_vals_filtered[i])
        if len(runtime_buffers['slide_window'][i]) >= CONFIG['WINDOW_SIZE']:
            win = list(runtime_buffers['slide_window'][i])
            x = np.arange(len(win))
            slope, _ = np.polyfit(x, win, 1)
            std_val = np.std(win)
            t_pos = algo_params['thr_slope_pos'][i]
            t_neg = algo_params['thr_slope_neg'][i]
            t_std = algo_params['thr_std'][i]
            is_triggered = False
            if slope > t_pos and std_val > t_std: is_triggered = True
            elif slope < t_neg and std_val > t_std: is_triggered = True
            if is_triggered:
                runtime_buffers['slip_counters'][i] += 1
            else:
                runtime_buffers['slip_counters'][i] = 0
            if runtime_buffers['slip_counters'][i] >= CONFIG['SLIP_COUNT_THRESHOLD']:
                flags[i] = 1
    return flags

# ================= 主程序 =================

if not os.path.exists(SAVE_DIR):
    try:
        os.makedirs(SAVE_DIR)
        print(f"Directory created: {SAVE_DIR}")
    except Exception as e:
        print(f"Error creating directory: {e}")
        SAVE_DIR = os.getcwd() 
else:
    print(f"Saving data to: {SAVE_DIR}")

# --- 1. 初始化电机 ---
print(f"Connecting to Motor on {MOTOR_PORT}...")
try:
    Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    serial_device = serial.Serial(MOTOR_PORT, MOTOR_BAUDRATE, timeout=0.5)
    MotorControl1 = MotorControl(serial_device)
    MotorControl1.addMotor(Motor1)
except Exception as e:
    print(f"Error connecting to Motor: {e}")
    exit()

# --- 2. 初始化 Arduino ---
print(f"Connecting to Arduino on {ARDUINO_PORT}...")
arduino_serial = None
try:
    arduino_serial = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.05)
    time.sleep(1)
    arduino_serial.reset_input_buffer()
    print("Arduino Connected.")
except Exception as e:
    print(f"Warning: Could not connect to Arduino on {ARDUINO_PORT}. Error: {e}")

# --- 电机参数设置 ---
if MotorControl1.switchControlMode(Motor1, Control_Type.VEL):
    print("Switch Control Mode to VEL: Success")

MotorControl1.save_motor_param(Motor1)
print("Enabling Motor...")
MotorControl1.enable(Motor1)
time.sleep(0.5)
MotorControl1.set_zero_position(Motor1)
time.sleep(0.5)
MotorControl1.refresh_motor_status(Motor1)
start_pos = Motor1.getPosition()
print(f"Motor Ready. Current Pos: {start_pos:.4f}")

# --- 3. 准备 CSV 文件 ---
timestamp_str = int(time.time())
motor_csv_path = os.path.join(SAVE_DIR, f"motor_data_{timestamp_str}.csv")
sensor_csv_path = os.path.join(SAVE_DIR, f"sensor_data_{timestamp_str}.csv")

motor_csv_file = open(motor_csv_path, mode='w', newline='', buffering=1)
motor_csv_writer = csv.writer(motor_csv_file)
motor_csv_writer.writerow(["Timestamp", "Position", "Velocity", "Raw_Torque", "Filtered_Torque", "State"])

sensor_csv_file = open(sensor_csv_path, mode='w', newline='', buffering=1)
sensor_csv_writer = csv.writer(sensor_csv_file)

sensor_headers = ["Timestamp", "Res1", "Res2", "Res3", "Res4", "Res5", 
                  "Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z",
                  "Proc_R1", "Proc_R2", "Proc_R3", "Proc_R4", "Proc_R5",
                  "Slip_1", "Slip_2", "Slip_3", "Slip_4", "Slip_5", 
                  "Algo_State", "Global_Slip",
                  "IMU_Slip_X", "IMU_Slip_Y", "IMU_Slip_Z"]
sensor_csv_writer.writerow(sensor_headers)

print(f"Motor File:  {motor_csv_path}")
print(f"Sensor File: {sensor_csv_path}")

# --- 4. 准备 实时绘图 (5路电阻) ---
print("Initializing Sensor Plot...")
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

lines = []
colors = ['r', 'g', 'b', 'c', 'm']
for i in range(5):
    ln, = ax.plot([], [], color=colors[i], label=f'Ch{i+1}', linewidth=1.5, alpha=0.8)
    lines.append(ln)

ax.set_title('Real-time Sensor Signals (Proc_R) & Slip Detection')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Delta R / R0')
ax.legend(loc='upper left')
ax.grid(True)
ax.set_ylim(-0.1, 0.1) 

# --- 绘图缓存区 ---
MAX_PLOT_LEN = 300
plot_time_buffer = deque(maxlen=MAX_PLOT_LEN)
plot_proc_res_buffers = [deque(maxlen=MAX_PLOT_LEN) for _ in range(5)]

# 1. 电阻检测到的真滑移缓存
plot_slip_event_buffer = deque(maxlen=MAX_PLOT_LEN) 
# 2. [新增] IMU检测到的真滑移缓存
plot_imu_slip_buffer = deque(maxlen=MAX_PLOT_LEN)   

vlines_collection = None      # 黑色虚线 (电阻滑移)
vlines_imu_collection = None  # [新增] 红色竖线 (IMU滑移)

last_plot_time = time.time()

# --- 变量初始化 ---
LPF_ALPHA = 0.1 
last_filtered_torque = 0.0
is_first_sample = True

CHECK_WINDOW_DURATION = 0.15
torque_check_start_time = None
torque_check_buffer = []
TORQUE_LIMIT = 0.4

STATE_IDLE = 0
STATE_FORWARD = 1
STATE_RETURN = 2
STATE_HOLD = 3  
control_state = STATE_IDLE

start_time = time.time()
last_w_pressed = False
last_s_pressed = False
ignore_torque_until = 0 

print("\nControls:\n  [W] - Init Sensor (4s) -> Then Move Forward\n  [S] - Return & Reset\n  [ESC] - Exit")

try:
    while True:
        # --- 1. 键盘监听 ---
        if keyboard.is_pressed('esc'):
            print("\nESC pressed. Exiting...")
            break
        
        is_w_down = keyboard.is_pressed('w')
        is_s_down = keyboard.is_pressed('s')

        # [W] 前进
        if is_w_down and not last_w_pressed:
            if control_state != STATE_FORWARD and control_state != STATE_HOLD:
                print("\n[W] Pressed. Phase 1: Sensor Init (Motor Idle)...")
                control_state = STATE_FORWARD
                
                if arduino_serial and arduino_serial.is_open:
                    arduino_serial.reset_input_buffer()
                reset_algorithm()
                sensing_state = SENSING_INIT
                sensing_start_time = current_sys_time 
                
                torque_check_start_time = None
                torque_check_buffer = []
        
        # [S] 返回
        if is_s_down and not last_s_pressed:
            if control_state != STATE_RETURN:
                print("\n[S] Return. Algorithm Reset.")
                control_state = STATE_RETURN
                reset_algorithm() 
                ignore_torque_until = time.time() + 0.5
                torque_check_start_time = None
                torque_check_buffer = []

        last_w_pressed = is_w_down
        last_s_pressed = is_s_down

        # --- 2. 获取电机数据 ---
        current_pos = Motor1.getPosition()
        current_vel = Motor1.getVelocity()
        current_torque = Motor1.getTorque()
        current_sys_time = time.time()
        current_time_log = current_sys_time - start_time

        in_grace_period = current_sys_time < ignore_torque_until

        if is_first_sample:
            filtered_torque = current_torque
            last_filtered_torque = current_torque
            is_first_sample = False
        else:
            filtered_torque = (LPF_ALPHA * current_torque) + ((1 - LPF_ALPHA) * last_filtered_torque)
            last_filtered_torque = filtered_torque

        # --- 3. 算法处理 ---
        if arduino_serial and arduino_serial.is_open:
            try:
                if arduino_serial.in_waiting > 0:
                    line_data = arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line_data:
                        parts = line_data.split(',')
                        if len(parts) == 11:
                            raw_res = list(map(float, parts[:5])) 
                            raw_imu = list(map(float, parts[5:8])) 
                            
                            proc_vals = [0.0] * 5
                            slip_flags = [0] * 5
                            is_global_slip = 0
                            imu_slip_flags = [0, 0, 0]
                            is_imu_triggered = 0 # IMU 是否触发了真滑移

                            # A. 初始化阶段 (INIT)
                            if sensing_state == SENSING_INIT:
                                for i in range(5):
                                    runtime_buffers['init_buffer'][i].append(raw_res[i])
                                for i in range(3):
                                    runtime_buffers['init_acc_buffer'][i].append(raw_imu[i])
                                
                                if current_sys_time - sensing_start_time >= CONFIG['INIT_DURATION']:
                                    debug_log("Init Finished. Computing Params...")
                                    compute_calibration_params()
                                    sensing_state = SENSING_MONITOR
                                    print(f"\n[Algo] Phase 2: Motor Start Moving!")
                                    ignore_torque_until = current_sys_time + 0.5

                            # B. 监测阶段 (MONITOR)
                            elif sensing_state == SENSING_MONITOR:
                                # (1) 电阻检测
                                for i in range(5):
                                    if algo_params['R0'][i] is not None and algo_params['R0'][i] != 0:
                                        delta = (raw_res[i] - algo_params['R0'][i]) / algo_params['R0'][i]
                                    else:
                                        delta = 0.0
                                    val_filt = ema_filter(delta, i)
                                    proc_vals[i] = val_filt
                                
                                slip_flags = check_for_slip(proc_vals)
                                if sum(slip_flags) >= 2:
                                    is_global_slip = 1
                                
                                # (2) IMU 检测
                                for i in range(3):
                                    if algo_params['Acc0'][i] is not None:
                                        acc_diff = abs(raw_imu[i] - algo_params['Acc0'][i])
                                        if acc_diff > CONFIG['ACC_SLIP_THRESHOLD']:
                                            imu_slip_flags[i] = 1
                                
                                # 只要任意一个轴触发，就认为发生了物理滑移
                                if sum(imu_slip_flags) > 0:
                                    is_imu_triggered = 1

                            # C. 记录数据
                            sensor_row = [f"{current_time_log:.4f}"] + parts + \
                                         [f"{v:.4f}" for v in proc_vals] + \
                                         slip_flags + \
                                         [sensing_state, is_global_slip] + \
                                         imu_slip_flags
                            sensor_csv_writer.writerow(sensor_row)

                            # D. 更新绘图 Buffer
                            plot_time_buffer.append(current_time_log)
                            for i in range(5):
                                plot_proc_res_buffers[i].append(proc_vals[i])
                            
                            plot_slip_event_buffer.append(is_global_slip) # 电阻滑移
                            plot_imu_slip_buffer.append(is_imu_triggered) # [新增] IMU 滑移

            except Exception as e:
                pass 
        
        # --- 4. 电机控制逻辑 ---
        if control_state == STATE_FORWARD:
            if sensing_state == SENSING_INIT:
                MotorControl1.control_Vel(Motor1, 0)
                rem = CONFIG['INIT_DURATION'] - (current_sys_time - sensing_start_time)
                if rem > 0 and int(rem*10)%5 == 0:
                     print(f"Calibrating... {rem:.1f}s", end='\r')

            elif sensing_state == SENSING_MONITOR:
                if not in_grace_period:
                    if torque_check_start_time is not None:
                        torque_check_buffer.append(filtered_torque)
                        if (current_sys_time - torque_check_start_time) >= CHECK_WINDOW_DURATION:
                            avg_torque = sum(torque_check_buffer) / len(torque_check_buffer)
                            
                            if avg_torque >= TORQUE_LIMIT:
                                print(f"\n[Trigger] Grip Detected (Torque > {TORQUE_LIMIT}). Holding.")
                                control_state = STATE_HOLD 
                                MotorControl1.control_Vel(Motor1, 0)
                            
                            torque_check_start_time = None
                            torque_check_buffer = []
                    elif filtered_torque >= TORQUE_LIMIT:
                        torque_check_start_time = current_sys_time
                        torque_check_buffer = [filtered_torque]
                    
                    if control_state == STATE_FORWARD:
                        MotorControl1.control_Vel(Motor1, 0.6)
                else:
                    MotorControl1.control_Vel(Motor1, 0.6)
                    torque_check_start_time = None

        elif control_state == STATE_HOLD:
            MotorControl1.control_Vel(Motor1, 0)
        
        elif control_state == STATE_RETURN:
            torque_check_start_time = None
            if abs(current_pos) <= 0.05: 
                print(f"\n[Trigger] Returned to Zero. Stopping.")
                control_state = STATE_IDLE
                MotorControl1.control_Vel(Motor1, 0)
            else:
                MotorControl1.control_Vel(Motor1, -0.6)
        
        else: # IDLE
            MotorControl1.control_Vel(Motor1, 0)
            torque_check_start_time = None

        # --- 5. 记录电机数据 ---
        state_str = ["IDLE", "FWD", "RET", "HOLD"][control_state]
        motor_csv_writer.writerow([f"{current_time_log:.4f}", current_pos, current_vel, current_torque, filtered_torque, state_str])

        # --- 6. 绘图更新 ---
        if current_sys_time - last_plot_time > 0.1:
            # 更新曲线
            for i in range(5):
                lines[i].set_xdata(plot_time_buffer)
                lines[i].set_ydata(plot_proc_res_buffers[i])
            
            # Y轴动态范围
            all_current_data = []
            for buf in plot_proc_res_buffers:
                all_current_data.extend(list(buf))
            
            if all_current_data:
                y_min = min(all_current_data)
                y_max = max(all_current_data)
                span = y_max - y_min
                if span < 0.002: 
                    center = (y_max + y_min) / 2
                    ax.set_ylim(center - 0.05, center + 0.05)
                else:
                    margin = span * 0.1
                    ax.set_ylim(y_min - margin, y_max + margin)

            # X轴范围
            if len(plot_time_buffer) > 0:
                ax.set_xlim(min(plot_time_buffer), max(plot_time_buffer) + 0.5)
            
            # === 绘制黑色虚线 (电阻滑移) ===
            if vlines_collection:
                vlines_collection.remove()
            
            slip_times = [t for t, slip in zip(plot_time_buffer, plot_slip_event_buffer) if slip == 1]
            ymin_curr, ymax_curr = ax.get_ylim()
            
            if slip_times:
                vlines_collection = ax.vlines(slip_times, ymin_curr, ymax_curr, colors='k', linestyles='dashed', alpha=0.5, linewidth=1, label='Res_Slip')
            else:
                vlines_collection = None

            # === [新增] 绘制红色竖线 (IMU滑移) ===
            if vlines_imu_collection:
                vlines_imu_collection.remove()

            imu_slip_times = [t for t, imu_s in zip(plot_time_buffer, plot_imu_slip_buffer) if imu_s == 1]
            
            if imu_slip_times:
                vlines_imu_collection = ax.vlines(imu_slip_times, ymin_curr, ymax_curr, colors='r', linestyles='solid', alpha=0.6, linewidth=1.5, label='IMU_Slip')
            else:
                vlines_imu_collection = None

            fig.canvas.flush_events() 
            last_plot_time = current_sys_time

except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected.")

finally:
    print("\nClosing resources...")
    MotorControl1.control_Vel(Motor1, 0)
    serial_device.close()
    if arduino_serial and arduino_serial.is_open:
        arduino_serial.close()
    motor_csv_file.close()
    sensor_csv_file.close()
    plt.ioff()
    plt.show()
    print(f"Files saved in: {SAVE_DIR}")