#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
光纤拉锥系统：双电机 + RS485继电器一体化控制程序

推荐流程：
1. 程序运行后，按回车键开始；
2. 等待 DELAY_BEFORE_HEAT 秒；
3. 使能两台电机；
4. 开启继电器；
5. 可选预热 PREHEAT_DURATION 秒；
6. 两台电机同步开始拉制；
7. 从电机同步触发开始计时，HEAT_AFTER_MOVE_DURATION 秒后立即关闭继电器；
8. 电机继续运动直到到位；
9. 再次按回车，两台电机同步回零。


- 到达加热截止时间后，立即发送 relay_off；
- finally 中无论如何都再次发送关闭继电器命令，防止高压模块残留开启。
"""

import serial
import time
import sys


# ============================================================
# 1. 串口配置
# ============================================================

RELAY_PORT = "COM5"          # 继电器串口
MOTOR_PORT = "COM4"          # 双电机串口

RELAY_BAUDRATE = 115200
MOTOR_BAUDRATE = 115200

SERIAL_TIMEOUT = 0.2         # 不建议设太大，否则读响应会拖慢程序
WRITE_TIMEOUT = 0.2


# ============================================================
# 2. 继电器 / 加热配置
# ============================================================

RELAY_ADDR = 0x03             # 继电器地址
RELAY_CHANNEL = 0             # 继电器通道

DELAY_BEFORE_HEAT = 3.0       # 按回车后等待多久开始
PREHEAT_DURATION =5.0        # 预热时长
HEAT_AFTER_MOVE_DURATION = 1  # 电机开始运动后，加热持续时间，单位 s

RELAY_RESPONSE_WAIT = 0.08    # 继电器响应最长时间
STATUS_POLL_INTERVAL = 0.05   # 电机状态轮询间隔

ENABLE_MOTOR_BEFORE_HEAT = True


# ============================================================
# 3. 电机配置
# ============================================================

CHECKSUM = 0x6B

ADDRESS_1 = 0x01
ADDRESS_2 = 0x02

LEAD = 12.0                   # 丝杆螺距，mm/圈
STEPS_PER_REV = 200           # 1.8°电机：200整步/圈

# 注意：
# 必须驱动器中的细分数一致。
MICROSTEPS = 256

PULSES_PER_REV = STEPS_PER_REV * MICROSTEPS

ACC = 0
DIRECTION = 0x01              # 按实际拉伸方向修改：0x00 或 0x01

SYNC_CMD = bytearray([0x00, 0xFF, 0x66, CHECKSUM])


# ============================================================
# 4. 回零配置
# ============================================================

ZERO_SPEED_RPM = 150
ZERO_ACC = 200
ZERO_SYNC = True
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

# ============================================================
# 5. 运动段配置
# revs: 运动圈数
# linear_speed:运动速度
# ============================================================

MOVEMENT_SEGMENTS = [
    {
        "name": "拉锥运动",
        "revs": 1,
        "linear_speed": 3,
    },
]


# ============================================================
# 6. Modbus RTU 继电器函数
# ============================================================

def calc_crc16(data: bytes) -> int:
    """计算 Modbus RTU CRC16，返回 int，发送时低字节在前。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_relay_command(addr: int, func: int, *args: int) -> bytes:
    """构建继电器 Modbus RTU 命令帧。"""
    frame = bytes([addr, func]) + bytes(args)
    crc = calc_crc16(frame)
    return frame + crc.to_bytes(2, byteorder="little")


def relay_read_short_response(ser: serial.Serial, max_wait: float = RELAY_RESPONSE_WAIT) -> bytes:
    """
    短时间读取继电器响应。
    不使用 ser.read(8) 长时间阻塞，避免影响关断时序。
    """
    resp = b""
    deadline = time.perf_counter() + max_wait

    while time.perf_counter() < deadline:
        n = ser.in_waiting
        if n > 0:
            resp += ser.read(n)
            if len(resp) >= 8:
                break
        time.sleep(0.005)

    return resp


def relay_on(ser: serial.Serial, channel: int = RELAY_CHANNEL) -> bytes:
    """开启指定继电器通道。"""
    cmd = build_relay_command(RELAY_ADDR, 0x05, 0x00, channel, 0xFF, 0x00)

    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()

    resp = relay_read_short_response(ser)

    print(f"继电器开启命令: {cmd.hex().upper()}")
    if resp:
        print(f"继电器开启响应: {resp.hex().upper()}")
    else:
        print("提示：未收到继电器开启响应，但开启命令已发送。")

    return resp


def relay_off(ser: serial.Serial, channel: int = RELAY_CHANNEL) -> bytes:
    """关闭指定继电器通道。"""
    cmd = build_relay_command(RELAY_ADDR, 0x05, 0x00, channel, 0x00, 0x00)

    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()

    
    resp = relay_read_short_response(ser)

    print(f"继电器关闭命令: {cmd.hex().upper()}")
    if resp:
        print(f"继电器关闭响应: {resp.hex().upper()}")
    else:
        print("提示：未收到继电器关闭响应，但关闭命令已发送。")

    return resp


def force_relay_off(ser: serial.Serial, channel: int = RELAY_CHANNEL, repeat: int = 2) -> None:
    """安全兜底：重复发送关闭命令。"""
    for i in range(repeat):
        try:
            print(f"安全关闭继电器，第 {i + 1} 次...")
            relay_off(ser, channel)
            time.sleep(0.05)
        except Exception as e:
            print(f"警告：继电器关闭命令发送失败：{e}", file=sys.stderr)


# ============================================================
# 7. 电机控制函数
# ============================================================

def motor_send_and_wait(
    ser: serial.Serial,
    cmd_bytes: bytes,
    wait: float = 0.03,
    print_reply: bool = True,
    clear_input: bool = True,
) -> bytes:
    """发送电机命令并短时间读取返回。"""
    if clear_input:
        ser.reset_input_buffer()

    ser.write(cmd_bytes)
    ser.flush()

    time.sleep(wait)

    resp = b""
    n = ser.in_waiting
    if n > 0:
        resp = ser.read(n)

    if resp and print_reply:
        print(f"电机回复: {resp.hex().upper()}")

    return resp


def enable_motor(ser: serial.Serial, addr: int, enable: bool = True) -> None:
    """使能 / 失能指定电机。"""
    cmd = bytearray([
        addr,
        0xF3,
        0xAB,
        0x01 if enable else 0x00,
        0x00,
        CHECKSUM,
    ])

    print(f"{'使能' if enable else '失能'}电机 {addr}: {cmd.hex().upper()}")
    motor_send_and_wait(ser, cmd, wait=0.05, print_reply=True)


def check_motor_status(ser: serial.Serial, addr: int):
    """
    读取电机状态。
    原代码假设返回帧第3个字节 resp[2] 为状态字节，bit1，即 0x02，表示到位。
    """
    cmd = bytearray([addr, 0x3A, CHECKSUM])
    resp = motor_send_and_wait(
        ser,
        cmd,
        wait=0.03,
        print_reply=False,
        clear_input=True,
    )

    if resp and len(resp) >= 3:
        return resp[2]

    return None


def is_position_reached(status) -> bool:
    """判断是否到位。"""
    return status is not None and bool(status & 0x02)


def rpm_from_linear_speed(linear_speed_mm_s: float) -> int:
    """线速度 mm/s 转电机转速 RPM。"""
    rpm = (linear_speed_mm_s * 60.0) / LEAD
    return int(round(rpm))


def pulses_from_revs(revs: float) -> int:
    """圈数转脉冲数。"""
    return int(round(revs * PULSES_PER_REV))


def build_position_cmd(
    addr: int,
    pulses: int,
    speed_rpm: int,
    acc: int,
    direction: int,
    absolute: bool = False,
    sync_enable: bool = True,
) -> bytearray:
    """
    构建电机位置模式命令。
    absolute=False：相对位置模式；
    absolute=True：绝对位置模式；
    sync_enable=True：预装参数，等待广播同步触发。
    """
    speed_hex = (int(speed_rpm) & 0xFFFF).to_bytes(2, "big")

    mode = 0x01 if absolute else 0x00
    sync_flag = 0x01 if sync_enable else 0x00

    return bytearray([
        addr,
        0xFD,
        direction,
        speed_hex[0],
        speed_hex[1],
        acc,
        (pulses >> 24) & 0xFF,
        (pulses >> 16) & 0xFF,
        (pulses >> 8) & 0xFF,
        pulses & 0xFF,
        mode,
        sync_flag,
        CHECKSUM,
    ])


def send_sync_trigger(ser: serial.Serial) -> float:
    """
    发送同步触发命令。
    返回触发时刻，用于精确计算加热关闭时间。
    """
    print(f"发送同步触发命令: {SYNC_CMD.hex().upper()}")

    ser.reset_input_buffer()
    ser.write(SYNC_CMD)
    ser.flush()

    motion_start_time = time.perf_counter()

    time.sleep(0.03)
    n = ser.in_waiting
    if n > 0:
        resp = ser.read(n)
        print(f"同步触发回复: {resp.hex().upper()}")

    return motion_start_time


def start_sync_segment(
    ser: serial.Serial,
    name: str,
    revs: float,
    linear_speed: float,
    acc: int = ACC,
    direction: int = DIRECTION,
):
    """
    只负责发送两台电机的运动参数并同步启动。
    不在这里阻塞等待到位。
    """
    if revs <= 0:
        raise ValueError(f"{name} 的 revs 必须大于 0")
    if linear_speed <= 0:
        raise ValueError(f"{name} 的 linear_speed 必须大于 0")

    pulses = pulses_from_revs(revs)
    speed_rpm = rpm_from_linear_speed(linear_speed)
    distance_mm = revs * LEAD
    theoretical_time = distance_mm / linear_speed

    timeout = max(60.0, theoretical_time * 1.8 + 20.0)

    print("-" * 60)
    print(
        f"{name}: 圈数={revs} 圈, 位移≈{distance_mm:.3f} mm, "
        f"速度={linear_speed} mm/s, RPM={speed_rpm}, 脉冲={pulses}"
    )
    print(f"理论运动时间≈{theoretical_time:.2f} s, 到位等待超时={timeout:.1f} s")

    cmd1 = build_position_cmd(
        ADDRESS_1,
        pulses,
        speed_rpm,
        acc,
        direction,
        absolute=False,
        sync_enable=True,
    )

    cmd2 = build_position_cmd(
        ADDRESS_2,
        pulses,
        speed_rpm,
        acc,
        direction,
        absolute=False,
        sync_enable=True,
    )

    print(f"发送电机1预装命令: {cmd1.hex().upper()}")
    motor_send_and_wait(ser, cmd1, wait=0.05, print_reply=True)

    print(f"发送电机2预装命令: {cmd2.hex().upper()}")
    motor_send_and_wait(ser, cmd2, wait=0.05, print_reply=True)

    motion_start_time = send_sync_trigger(ser)

    return {
        "name": name,
        "timeout": timeout,
        "theoretical_time": theoretical_time,
        "motion_start_time": motion_start_time,
    }


def wait_segment_done_and_control_heater(
    motor_ser: serial.Serial,
    relay_ser: serial.Serial,
    segment_info: dict,
    heater_state: dict,
    heat_off_deadline: float,
) -> None:
    """
    等待当前运动段完成，同时负责准时关闭继电器。

    关键逻辑：
    - 循环中每次先判断是否到达 heat_off_deadline；
    - 到点立即 relay_off；
    - 之后继续轮询电机状态；
    - 电机到位后退出；
    - 如果电机提前到位，也立即关闭继电器。
    """
    name = segment_info["name"]
    timeout = segment_info["timeout"]
    segment_start_time = time.perf_counter()

    motor1_done = False
    motor2_done = False

    while time.perf_counter() - segment_start_time < timeout:
        now = time.perf_counter()

        # 1. 到达加热截止时间，立即关闭继电器
        if heater_state.get("on", False) and now >= heat_off_deadline:
            elapsed = now - segment_info["motion_start_time"]
            print(f"\n电机开始运动后已加热 {elapsed:.3f} s，立即关闭继电器。")
            relay_off(relay_ser, RELAY_CHANNEL)
            heater_state["on"] = False

        # 2. 轮询电机到位状态
        if not motor1_done:
            status1 = check_motor_status(motor_ser, ADDRESS_1)
            if is_position_reached(status1):
                motor1_done = True
                print("电机 1 已到位")

        if not motor2_done:
            status2 = check_motor_status(motor_ser, ADDRESS_2)
            if is_position_reached(status2):
                motor2_done = True
                print("电机 2 已到位")

        # 3. 两台电机均到位，运动段结束
        if motor1_done and motor2_done:
            print(f"{name} 完成")
            if heater_state.get("on", False):
                print("\n电机已提前到位，立即关闭继电器。")
                relay_off(relay_ser, RELAY_CHANNEL)
                heater_state["on"] = False
            return

        time.sleep(STATUS_POLL_INTERVAL)

    # 超时也必须先关继电器
    if heater_state.get("on", False):
        print("\n电机等待到位超时，安全关闭继电器。")
        relay_off(relay_ser, RELAY_CHANNEL)
        heater_state["on"] = False

    raise TimeoutError(f"{name} 未确认两台电机均到位，请检查电机状态。")


def wait_for_position_reached(ser: serial.Serial, addr: int, timeout: float = 120.0) -> bool:
    """用于回零阶段的简单阻塞等待。"""
    start = time.perf_counter()

    while time.perf_counter() - start < timeout:
        status = check_motor_status(ser, addr)
        if is_position_reached(status):
            print(f"电机 {addr} 已到位")
            return True
        time.sleep(0.1)

    print(f"警告：电机 {addr} 等待到位超时")
    return False


def go_to_zero_single(
    ser: serial.Serial,
    addr: int,
    speed_rpm: int = ZERO_SPEED_RPM,
    acc: int = ZERO_ACC,
) -> None:
    """单台电机绝对位置回零。"""
    cmd = build_position_cmd(
        addr,
        0,
        speed_rpm,
        acc,
        direction=0x00,
        absolute=True,
        sync_enable=False,
    )

    print(f"发送电机 {addr} 绝对位置回零命令: {cmd.hex().upper()}")
    motor_send_and_wait(ser, cmd, wait=0.05, print_reply=True)

    wait_for_position_reached(ser, addr, timeout=120.0)


def go_to_zero_sync(
    ser: serial.Serial,
    speed_rpm: int = ZERO_SPEED_RPM,
    acc: int = ZERO_ACC,
) -> None:
    """两台电机同步绝对位置回零。"""
    cmd1 = build_position_cmd(
        ADDRESS_1,
        0,
        speed_rpm,
        acc,
        direction=0x00,
        absolute=True,
        sync_enable=True,
    )

    cmd2 = build_position_cmd(
        ADDRESS_2,
        0,
        speed_rpm,
        acc,
        direction=0x00,
        absolute=True,
        sync_enable=True,
    )

    print("-" * 60)
    print("两台电机同步回零")

    print(f"发送电机1回零预装命令: {cmd1.hex().upper()}")
    motor_send_and_wait(ser, cmd1, wait=0.05, print_reply=True)

    print(f"发送电机2回零预装命令: {cmd2.hex().upper()}")
    motor_send_and_wait(ser, cmd2, wait=0.05, print_reply=True)

    send_sync_trigger(ser)

    wait_for_position_reached(ser, ADDRESS_1, timeout=120.0)
    wait_for_position_reached(ser, ADDRESS_2, timeout=120.0)

    print("两台电机已回零")


# ============================================================
# 8. 主流程
# ============================================================

def run_process() -> None:
    relay_ser = None
    motor_ser = None

    heater_state = {
        "on": False,
    }

    try:
        print("正在打开串口...")

        relay_ser = serial.Serial(
            RELAY_PORT,
            RELAY_BAUDRATE,
            timeout=SERIAL_TIMEOUT,
            write_timeout=WRITE_TIMEOUT,
        )

        motor_ser = serial.Serial(
            MOTOR_PORT,
            MOTOR_BAUDRATE,
            timeout=SERIAL_TIMEOUT,
            write_timeout=WRITE_TIMEOUT,
        )

        time.sleep(0.2)

        print(f"继电器串口已打开: {RELAY_PORT}")
        print(f"电机串口已打开: {MOTOR_PORT}")

        # 程序启动后，先发一次关闭，保证继电器初始为断开状态
        print("\n初始化安全检查：先关闭继电器一次。")
        force_relay_off(relay_ser, RELAY_CHANNEL, repeat=1)
        heater_state["on"] = False

        input("\n请确认光纤已夹紧、加热模块位置正确。按回车键开始流程...")

        print(f"\n等待 {DELAY_BEFORE_HEAT} s 后开始流程...")
        time.sleep(DELAY_BEFORE_HEAT)

        if ENABLE_MOTOR_BEFORE_HEAT:
            print("\n加热前使能两台电机，使滑台保持当前位置...")
            enable_motor(motor_ser, ADDRESS_1, True)
            time.sleep(0.05)
            enable_motor(motor_ser, ADDRESS_2, True)
            time.sleep(0.05)

        print("\n开启继电器，高压电弧模块开始加热...")
        relay_on(relay_ser, RELAY_CHANNEL)
        heater_state["on"] = True

        if PREHEAT_DURATION > 0:
            print(f"预加热 {PREHEAT_DURATION} s，电机暂不运动...")
            time.sleep(PREHEAT_DURATION)
        else:
            print("未设置预热，继电器开启后将立即开始拉制。")

        if not ENABLE_MOTOR_BEFORE_HEAT:
            print("\n使能两台电机...")
            enable_motor(motor_ser, ADDRESS_1, True)
            time.sleep(0.05)
            enable_motor(motor_ser, ADDRESS_2, True)
            time.sleep(0.05)

        print(
            f"\n开始拉制：从电机同步触发开始计时，"
            f"{HEAT_AFTER_MOVE_DURATION} s 后关闭继电器，电机继续运动到位。"
        )

        first_segment = True
        heat_off_deadline = None

        for seg in MOVEMENT_SEGMENTS:
            segment_info = start_sync_segment(
                motor_ser,
                name=str(seg.get("name", "运动段")),
                revs=float(seg["revs"]),
                linear_speed=float(seg["linear_speed"]),
                acc=ACC,
                direction=DIRECTION,
            )

            if first_segment:
                heat_off_deadline = (
                    segment_info["motion_start_time"] + HEAT_AFTER_MOVE_DURATION
                )
                first_segment = False

            wait_segment_done_and_control_heater(
                motor_ser=motor_ser,
                relay_ser=relay_ser,
                segment_info=segment_info,
                heater_state=heater_state,
                heat_off_deadline=heat_off_deadline,
            )

        # 所有运动段完成后再次确认关闭
        if heater_state.get("on", False):
            print("\n所有运动段已完成，保险关闭继电器。")
            relay_off(relay_ser, RELAY_CHANNEL)
            heater_state["on"] = False

        print("\n运动完成，电机已停止，继电器应已关闭。")

        input("\n按回车键让两台电机回到零点...")

        if ZERO_SYNC:
            go_to_zero_sync(motor_ser, ZERO_SPEED_RPM, ZERO_ACC)
        else:
            go_to_zero_single(motor_ser, ADDRESS_1, ZERO_SPEED_RPM, ZERO_ACC)
            go_to_zero_single(motor_ser, ADDRESS_2, ZERO_SPEED_RPM, ZERO_ACC)

        print("\n流程完成，程序结束。")

    except KeyboardInterrupt:
        print("\n用户中断程序。")

    except serial.SerialException as e:
        print(f"串口错误: {e}", file=sys.stderr)

    except Exception as e:
        print(f"运行异常: {e}", file=sys.stderr)

    finally:
        # 任何情况下都优先关闭继电器
        if relay_ser is not None and relay_ser.is_open:
            try:
                print("\nfinally 安全保护：强制关闭继电器。")
                force_relay_off(relay_ser, RELAY_CHANNEL, repeat=2)
                heater_state["on"] = False
            except Exception as e:
                print(
                    f"警告：关闭继电器失败，请立即手动断开高压模块电源！错误: {e}",
                    file=sys.stderr,
                )

        if motor_ser is not None and motor_ser.is_open:
            motor_ser.close()
            print(f"电机串口 {MOTOR_PORT} 已关闭")

        if relay_ser is not None and relay_ser.is_open:
            relay_ser.close()
            print(f"继电器串口 {RELAY_PORT} 已关闭")


if __name__ == "__main__":
    run_process()