# -*- coding: utf-8 -*-
"""
Created on Wed Apr 29 16:31:48 2026

@author: thinkpad
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
光纤拉锥系统——高压电弧自动加热控制模块
通过 RS485 (Modbus RTU) 控制继电器启停，实现定时加热
继电器地址: 03, 波特率: 115200, 8N1
"""

import serial
import time
import sys

# ================= 用户可配置参数 =================
SERIAL_PORT = 'COM5'          # 根据实际端口修改 (Windows: COMx, Linux: /dev/ttyUSBx)
BAUDRATE = 115200
RELAY_ADDR = 0x03             # 继电器模块地址
CHANNEL = 0                   # 使用的继电器通道 (0~7)
DELAY_BEFORE_HEAT = 3         # 程序启动后延迟多久才开始加热 (秒)
HEAT_DURATION = 2            # 加热持续时间 (秒)
# =================================================

def calc_crc16(data: bytes) -> int:
    """
    计算 Modbus CRC16 校验码 (低字节在前)
    多项式: 0xA001 (反转的 0x8005)
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def build_command(addr, func, *args) -> bytes:
    """构建带 CRC 的 Modbus RTU 命令帧"""
    frame = bytes([addr, func]) + bytes(args)
    crc = calc_crc16(frame)
    # CRC 小端序添加
    frame += crc.to_bytes(2, byteorder='little')
    return frame

def relay_on(ser, channel=0):
    """开启指定继电器通道 (功能码 0x05)"""
    cmd = build_command(RELAY_ADDR, 0x05, 0x00, channel, 0xFF, 0x00)
    ser.write(cmd)
    time.sleep(0.05)
    # 读取响应 (继电器会原样返回命令)
    resp = ser.read(ser.in_waiting)
    if resp:
        print(f"开启响应: {resp.hex().upper()}")
    return resp

def relay_off(ser, channel=0):
    """关闭指定继电器通道"""
    cmd = build_command(RELAY_ADDR, 0x05, 0x00, channel, 0x00, 0x00)
    ser.write(cmd)
    time.sleep(0.05)
    resp = ser.read(ser.in_waiting)
    if resp:
        print(f"关闭响应: {resp.hex().upper()}")
    return resp

def main():
    print(f"串口 {SERIAL_PORT} 打开中...")
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print("串口已就绪")
            
            # 等待延迟后启动加热
            print(f"{DELAY_BEFORE_HEAT} 秒后自动开启加热...")
            time.sleep(DELAY_BEFORE_HEAT)
            
            # 开启继电器，启动高压电弧模块
            print(f"开启继电器通道 {CHANNEL}...")
            relay_on(ser, CHANNEL)
            
            # 持续加热指定时长
            print(f"加热中...持续 {HEAT_DURATION} 秒")
            time.sleep(HEAT_DURATION)
            
            # 关闭继电器，停止加热
            print("加热完成，关闭继电器...")
            relay_off(ser, CHANNEL)
            
            print("操作成功，程序退出。")
            
    except serial.SerialException as e:
        print(f"串口错误: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"运行异常: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()