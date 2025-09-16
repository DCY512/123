# -*- coding: utf-8 -*-

import serial
import time

# 串口配置
SERIAL_PORT = "COM12"  # 替换为你的串口号
BAUD_RATE = 115200

# 需要发送的 AT 指令
at_commands = [
    "AT+CWMODE=1",
    'AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com "',
    'AT+CWJAP="DCY","12345678"',
    'AT+MQTTUSERCFG=0,1,"NULL","PYQT&iggisf93NrW","3870efe14bdc3ebccae0d6778404635cbcdb79eb06273b0224bdb1321eb5ffd6",0,0,""',
    'AT+MQTTCLIENTID=0,"iggisf93NrW.PYQT|securemode=2\\,signmethod=hmacsha256\\,timestamp=1741496379578|"',
    'AT+MQTTCONN=0,"iot-06z00emlemujeca.mqtt.iothub.aliyuncs.com",1883,1'
]

def send_at_commands(commands):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3)
        print(f"打开串口 {SERIAL_PORT}")
    except Exception as e:
        print(f"串口打开失败: {e}")
        return

    for cmd in commands:
        full_cmd = cmd + '\r\n'
        ser.write(full_cmd.encode())
        print(f"发送: {cmd}")

        # 读取串口回复直到包含 OK 或超时
        response = ""
        start_time = time.time()
        while True:
            if ser.in_waiting > 0:
                res = ser.read(ser.in_waiting).decode(errors='ignore')
                response += res
                print(f"收到: {res.strip()}")
                if "OK" in response or "ERROR" in response:
                    break
            elif time.time() - start_time > 10:  # 超时 10 秒
                print("等待回复超时，跳过此命令")
                break
            time.sleep(0.1)

        # 如果没有收到OK就不继续
        if "OK" not in response:
            print(f"指令失败: {cmd}")
            break
        time.sleep(1)  # 稍作等待后发下一条

    ser.close()
    print("所有指令处理完成，串口已关闭")

if __name__ == "__main__":
    send_at_commands(at_commands)
