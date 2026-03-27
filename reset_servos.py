import serial
import time

# ================= 配置区 =================
# 请在这里填写你的舵机ID
SERVO_ID_1 = 1  # 对应从 180 度转到 360 度的舵机
SERVO_ID_2 = 2  # 对应从 180 度转到 0 度的舵机

# 树莓派 Ubuntu 系统的串口设备名:
# 树莓派在 Ubuntu 下硬件串口通常被识别为 '/dev/ttyS0' (Mini UART) 或 '/dev/ttyAMA0'
# 如果使用 USB 转串口，通常为 '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyS0'
BAUDRATE = 115200

# ==========================================

def init_serial():
    """初始化串口连接"""
    try:
        # 在树莓派上使用 USB 串口通信通常是 /dev/ttyUSB0 或者 /dev/ttyACM0
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"串口 {SERIAL_PORT} 已成功连通。")
        return ser
    except Exception as e:
        print(f"串口打开发生异常，将仅打印指令: {e}")
        return None

def move_multiple_servos(ser, target_angles, move_time=2000):
    """
    控制多个舵机同时转动到指定角度的函数
    :param ser: 串口对象
    :param target_angles: 字典或列表，格式如 {舵机ID: 目标角度}
    :param move_time: 运动时间(毫秒)
    """
    # 多个舵机同时运动时，使用 {} 将多条指令包裹起来
    command = "{"
    for servo_id, angle in target_angles.items():
        # 将 0-360 度的角度映射为 0500-2500 的 PWM 值
        pwm = int(500 + (angle / 360.0) * 2000)
        pwm = max(500, min(2500, pwm))  # 限制范围
        command += f"#{servo_id:03d}P{pwm:04d}T{move_time:04d}!"
    command += "}\r\n"
    
    print(f"生成同步指令: {command.strip()}")
    
    if ser and ser.is_open:
        # 发送串行指令
        ser.write(command.encode('utf-8'))

def reset_servos(ser):
    print("开始执行舵机同步初始化...")
    
    # 将多个舵机的目标状态打包到一起
    # 舵机1：转到 360 度，舵机2：转到 0 度 (同时执行，耗时均为5000ms)
    targets = {
        SERVO_ID_1: 360,
        SERVO_ID_2: 0
    }
    
    move_multiple_servos(ser, targets, 5000)
    
    # 给予舵机5.5秒钟完成5000ms的运动
    time.sleep(5.5)
    print("舵机同步初始化完成！")

if __name__ == "__main__":
    serial_conn = init_serial()
    
    try:
        reset_servos(serial_conn)
    except KeyboardInterrupt:
        print("程序已手动终止")
    finally:
        if serial_conn and serial_conn.is_open:
            serial_conn.close()
            print("串口已安全释放")
