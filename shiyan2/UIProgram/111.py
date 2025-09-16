# -*- coding: utf-8 -*-
import serial


def read_serial_data(port, baudrate=9600):
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"已连接串口 {port}，波特率 {baudrate}")

    try:
        while True:
            if ser.in_waiting > 0:
                # 读取一行数据（假设以换行符结尾）
                raw_data = ser.readline().decode('ascii', errors='ignore').strip()
                print(f"原始数据：{raw_data}")

                # 解析数据
                try:
                    ranges = parse_serial_data(raw_data)
                    for idx, val in enumerate(ranges):
                        print(f"range[{idx}]={val}(mm)")
                except ValueError as e:
                    print(f"解析失败：{str(e)}")

    except KeyboardInterrupt:
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    # 根据实际情况修改串口号（Windows为COMx，Linux为/dev/ttyXXX）
    read_serial_data('/COM13')