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
    """ ·¢ËÍ AT Ö¸Áî£¬µÈ´ýÏìÓ¦£¬³É¹¦ºóÔÙÖ´ÐÐÏÂÒ»Ìõ£¬¼ä¸ô 5 Ãë """
    for attempt in range(retries):
        ser.write((command + "\r\n").encode())  # ·¢ËÍÖ¸Áî
        rospy.loginfo("Sent: {}".format(command))
        time.sleep(5)  # µÈ´ý 2 Ãë»ñÈ¡ÏìÓ¦

        response = ser.read_all().decode().strip()  # »ñÈ¡ÏìÓ¦
        if response:
            rospy.loginfo("Response: {}".format(response))
            time.sleep(delay)  # ³É¹¦ºóÔÙµÈ´ý 5 Ãë
            return response  # Èç¹ûÓÐÏìÓ¦£¬·µ»ØÏìÓ¦
        
        rospy.logwarn("No response for '{}', retrying {}/{}".format(command, attempt+1, retries))
    
    rospy.logerr("Failed to get response for '{}', skipping...".format(command))
    return None  # Èç¹ûÒ»Ö±Ã»ÓÐÏìÓ¦£¬·µ»Ø None

def at_command_callback(msg):
    """ ´¦Àí ROS »°ÌâÏûÏ¢£¬·¢ËÍ AT Ö¸Áî """
    command = msg.data
    rospy.loginfo("Received ROS command: {}".format(command))
    send_at_command(command)

def mqtt_listener():
    """ ¼àÌý´®¿ÚµÄ MQTT ÏûÏ¢£¬²¢×ª·¢µ½ ROS »°Ìâ """
    pub = rospy.Publisher("/mqtt_data", String, queue_size=10)
    forwarded_pub = rospy.Publisher("/forwarded_data", String, queue_size=10)
    while not rospy.is_shutdown():
        if ser.in_waiting:
            data = ser.readline().decode().strip()
            if data:
                rospy.loginfo("Received MQTT message: {}".format(data))
                pub.publish(data)  # ·¢²¼µ½ /mqtt_data
                forwarded_pub.publish(data)  # ×ª·¢µ½ /forwarded_data

def at_command_node():
    """ ROS ½ÚµãÖ÷º¯Êý """
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

    # °´ÕÕÄãµÄÐèÇó·¢ËÍ AT Ö¸Áî
    commands = [
    "AT+CWMODE=1",  # 脡猫脰脙WiFi脛拢脢陆脦陋Station
    "AT+CIPSNTPCFG=1,8,\"ntp1.aliyun.com \"",  # 脜盲脰脙SNTP路镁脦帽脝梅
    "AT+CWJAP=\"DCY\",\"12345678\"",  # 脕卢陆脫WiFi
    "AT+MQTTUSERCFG=0,1,\"NULL\",\"ROSPI&iggisf93NrW\",\"84636b794ea1a4d3307ccab810685b76b806eb2c63f04f3333dd84de81146318\",0,0,\"\"",
    "AT+MQTTCLIENTID=0,\"iggisf93NrW.ROSPI|securemode=2\,signmethod=hmacsha256\,timestamp=1741498772243|\"",
    "AT+MQTTCONN=0,\"iot-06z00emlemujeca.mqtt.iothub.aliyuncs.com\",1883,1",
    "AT+MQTTSUB=0,\"/iggisf93NrW/ROSPI/user/get\",1",
    "AT+MQTTPUB=0,\"/sys/iggisf93NrW/ROSPI/thing/event/property/post\",\"{\\\"params\\\":{\\\"DCYY\\\":322}}\",1,0"
]


    # ·¢ËÍÃ¿¸ö AT Ö¸Áî£¬È·ÈÏÓÐÏìÓ¦ºóÔÙÖ´ÐÐÏÂÒ»Ìõ£¬¼ä¸ô 5 Ãë
    for command in commands:
        response = send_at_command(command, delay=5)  # ÓÐÏìÓ¦ÔÙÖ´ÐÐÏÂÒ»Ìõ£¬²¢¼ä¸ô 5 Ãë
        if not response:
            rospy.logerr("Skipping command due to lack of response: {}".format(command))

    # ÔÚ·¢ËÍ AT+MQTTCONN ºóÔö¼ÓÑÓÊ±£¬È·±£Á¬½ÓÎÈ¶¨
    rospy.loginfo("Waiting for MQTT connection to complete...")
    time.sleep(10)  # Ôö¼Ó 10 ÃëµÄÑÓÊ±£¬È·±£Á¬½ÓÍê³É

    # ·¢ËÍ MQTT ¶©ÔÄÃüÁî
    response = send_at_command('AT+MQTTSUB=0,"/iggisf93NrW/ROSPI/user/get",1', delay=5)
    if not response:
        rospy.logerr("Failed to subscribe to MQTT topic.")

    # Æô¶¯ MQTT ÏûÏ¢¼àÌýÏß³Ì
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

