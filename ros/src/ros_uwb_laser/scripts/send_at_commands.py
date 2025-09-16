#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import time
import threading
from std_msgs.msg import String

SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 115200

def send_at_command(command, delay=5):  # ½«Ä¬ÈÏÑÓ³ÙÐÞ¸ÄÎª5Ãë
    ser.write((command + "\r\n").encode())
    time.sleep(delay)  # ÑÓ³Ù5Ãë
    response = ser.read_all().decode()
    rospy.loginfo("Sent: {}\nResponse: {}".format(command, response))
    return response

def at_command_callback(msg):
    command = msg.data
    rospy.loginfo("Received ROS command: {}".format(command))
    send_at_command(command)

def mqtt_listener():
    pub = rospy.Publisher("/mqtt_data", String, queue_size=10)
    while not rospy.is_shutdown():
        if ser.in_waiting:
            data = ser.readline().decode().strip()
            if data:
                rospy.loginfo("Received MQTT message: {}".format(data))
                pub.publish(data)

def at_command_node():
    rospy.init_node("at_command_node", anonymous=True)
    rospy.loginfo("AT Command Node started")

    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        rospy.loginfo("Connected to {} at {} baud".format(SERIAL_PORT, BAUD_RATE))
    except serial.SerialException as e:
        rospy.logerr("Serial connection failed: {}".format(e))
        return

    rospy.Subscriber("/at_command", String, at_command_callback)

    # ·¢ËÍATÖ¸ÁîÊ±¼ä¼ä¸ôÒÑÐÞ¸ÄÎª5Ãë
    send_at_command("AT+CWMODE=1")
    send_at_command('AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com"')
    send_at_command('AT+CWJAP="DCY","12345678"')

    send_at_command('AT+MQTTUSERCFG=0,1,"NULL","ROSPI&iggisf93NrW","84636b794ea1a4d3307ccab810685b76b806eb2c63f04f3333dd84de81146318",0,0,""')
    send_at_command('AT+MQTTCLIENTID=0,"iggisf93NrW.ROSPI|securemode=2,signmethod=hmacsha256,timestamp=1741498772243|"')
    send_at_command('AT+MQTTCONN=0,"iot-06z00emlemujeca.mqtt.iothub.aliyuncs.com",1883,1')

    send_at_command('AT+MQTTSUB=0,"/iggisf93NrW/ROSPI/user/get",1')

    listener_thread = threading.Thread(target=mqtt_listener)
    listener_thread.daemon = True
    listener_thread.start()

    rospy.loginfo("All AT commands sent, waiting for MQTT messages...")
    rospy.spin()

if __name__ == "__main__":
    try:
        at_command_node()
    except rospy.ROSInterruptException:
        pass

