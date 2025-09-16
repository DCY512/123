#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import serial
import time
import threading
from std_msgs.msg import String

SERIAL_PORT = "/dev/ttyUSB4"
BAUD_RATE = 115200

def send_at_command(command, retries=3, delay=3):
    """ ���0�4�0�9�0�1 AT �0�0�0�0�0�9�0�6�0�5�0�1�0�8�0�6�0�7�0�5�0�3���0�7�0�7�0�5�0�1�0�6�0�7�0�1�0�7�0�2���0�8�0�2�0�0�0�7�0�4�0�4�0�3�0�0�0�6�0�3�0�0�0�1�0�5�0�1�0�4�0�1�0�0�0�0 5 �0�1�0�5 """
    for attempt in range(retries):
        ser.write((command + "\r\n").encode())  # ���0�4�0�9�0�1�0�0�0�0�0�9�0�6
        rospy.loginfo("Sent: {}".format(command))
        time.sleep(5)  # �0�8�0�6�0�7�0�5 2 �0�1�0�5�0�3�0�9�0�6�0�3�0�3���0�7�0�7

        response = ser.read_all().decode().strip()  # �0�3�0�9�0�6�0�3�0�3���0�7�0�7
        if response:
            rospy.loginfo("Response: {}".format(response))
            time.sleep(delay)  # �0�6�0�7�0�1�0�7�0�2���0�8�0�2�0�8�0�6�0�7�0�5 5 �0�1�0�5
            return response  # �0�6�0�4�0�1�0�4�0�7�0�4�0�3���0�7�0�7�0�5�0�1���0�8�0�3�0�1�0�3���0�7�0�7
        
        rospy.logwarn("No response for '{}', retrying {}/{}".format(command, attempt+1, retries))
    
    rospy.logerr("Failed to get response for '{}', skipping...".format(command))
    return None  # �0�6�0�4�0�1�0�4�0�6�0�3�0�0���0�1�0�3�0�7�0�4�0�3���0�7�0�7�0�5�0�1���0�8�0�3�0�1 None

def at_command_callback(msg):
    """ �0�7�0�7�0�8�� ROS �0�3��0�0�0�9�0�3�0�4�0�3�0�4�0�5�0�1���0�4�0�9�0�1 AT �0�0�0�0�0�9�0�6 """
    command = msg.data
    rospy.loginfo("Received ROS command: {}".format(command))
    send_at_command(command)

def mqtt_listener():
    """ �0�4���0�0�0�5�0�7�0�3�0�7�0�3�0�8�0�2 MQTT �0�3�0�4�0�3�0�4�0�5�0�1�0�5�0�4���0�9���0�4�0�8�0�5 ROS �0�3��0�0�0�9 """
    pub = rospy.Publisher("/mqtt_data", String, queue_size=10)
    forwarded_pub = rospy.Publisher("/forwarded_data", String, queue_size=10)
    while not rospy.is_shutdown():
        if ser.in_waiting:
            data = ser.readline().decode().strip()
            if data:
                rospy.loginfo("Received MQTT message: {}".format(data))
                pub.publish(data)  # ���0�4�0�5�0�4�0�8�0�5 /mqtt_data
                forwarded_pub.publish(data)  # ���0�9���0�4�0�8�0�5 /forwarded_data

def at_command_node():
    """ ROS �0�5�0�3�0�8�0�0�0�0�0�2�0�4�0�8�0�5 """
    rospy.init_node("ros_uwb_laser", anonymous=True)
    rospy.loginfo("AT Command Node started")

    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        rospy.loginfo("Connected to {} at {} baud".format(SERIAL_PORT, BAUD_RATE))
    except serial.SerialException as e:
        rospy.logerr("Serial connection failed: {}".format(e))
        return

    rospy.Subscriber("/at_command", String, at_command_callback)

    # ��0�7�0�9�0�9�0�2�0�0�0�8�0�2�0�4���0�5�����0�4�0�9�0�1 AT �0�0�0�0�0�9�0�6
    commands = [
    "AT+CWMODE=1",  # ÉèÖÃWiFiÄ£Ê½ÎªStation
    "AT+CIPSNTPCFG=1,8,\"ntp1.aliyun.com \"",  # ÅäÖÃSNTP·þÎñÆ÷
    "AT+CWJAP=\"DCY\",\"12345678\"",  # Á¬½ÓWiFi
    "AT+MQTTUSERCFG=0,1,\"NULL\",\"ROSPI&iggisf93NrW\",\"84636b794ea1a4d3307ccab810685b76b806eb2c63f04f3333dd84de81146318\",0,0,\"\"",
    "AT+MQTTCLIENTID=0,\"iggisf93NrW.ROSPI|securemode=2\,signmethod=hmacsha256\,timestamp=1741498772243|\"",
    "AT+MQTTCONN=0,\"iot-06z00emlemujeca.mqtt.iothub.aliyuncs.com\",1883,1",
    "AT+MQTTSUB=0,\"/iggisf93NrW/ROSPI/user/get\",1",
    "AT+MQTTPUB=0,\"/sys/iggisf93NrW/ROSPI/thing/event/property/post\",\"{\\\"params\\\":{\\\"DCYY\\\":322}}\",1,0"
]


    # ���0�4�0�9�0�1�0�1�0�7�0�0�0�2 AT �0�0�0�0�0�9�0�6�0�5�0�1�0�6���0�6�0�3�0�7�0�4�0�3���0�7�0�7�0�2���0�8�0�2�0�0�0�7�0�4�0�4�0�3�0�0�0�6�0�3�0�0�0�1�0�5�0�1�0�4�0�1�0�0�0�0 5 �0�1�0�5
    for command in commands:
        response = send_at_command(command, delay=5)  # �0�7�0�4�0�3���0�7�0�7�0�8�0�2�0�0�0�7�0�4�0�4�0�3�0�0�0�6�0�3�0�0�0�1�0�5�0�1�0�5�0�4�0�4�0�1�0�0�0�0 5 �0�1�0�5
        if not response:
            rospy.logerr("Skipping command due to lack of response: {}".format(command))

    # �0�8�0�3���0�4�0�9�0�1 AT+MQTTCONN �0�2���0�8�0�2�0�4�0�7�0�5�0�7�0�8���0�5�0�1�0�6�����0�5�0�9�0�1�0�5�0�7�0�2�0�6�0�9��
    rospy.loginfo("Waiting for MQTT connection to complete...")
    time.sleep(10)  # �0�8�0�2�0�4�0�7 10 �0�1�0�5�0�8�0�2�0�5�0�7�0�8���0�5�0�1�0�6�����0�5�0�9�0�1�0�5�0�7�0�1���0�6�0�7

    # ���0�4�0�9�0�1 MQTT �0�9�0�8�0�8�0�2�0�1���0�9�0�6
    response = send_at_command('AT+MQTTSUB=0,"/iggisf93NrW/ROSPI/user/get",1', delay=5)
    if not response:
        rospy.logerr("Failed to subscribe to MQTT topic.")

    # �0�4�0�0�0�9�0�4 MQTT �0�3�0�4�0�3�0�4�0�4���0�0�0�5�0�3�0�8�0�6�0�0
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

