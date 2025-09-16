#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import math
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

# Hardware Configuration
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
HEADER_BYTE = b'\x11'
PACKET_SIZE = 78

def parse_raw_yaw(packet):
    """�0�0���0�5�0�7�0�5�0�9�0�2�0�2�0�8�0�2�0�8�0�4Yaw�0�5�0�5�0�9�0�6"""
    if len(packet) != PACKET_SIZE or packet[0] != HEADER_BYTE:
        return None
    
    try:
        # �0�5�0�9�0�2�0�2�0�9�0�2�0�8�0�9�0�8�0�5�0�8�0�5�0�6�0�6
        q = [
            struct.unpack('<h', packet[7+i*2:7+(i+1)*2])[0] * 0.000030517578125
            for i in range(4)
        ]
        
        # �0�4�0�4�0�9�0�0�0�8�0�2�0�8�0�4�0�3���0�8�0�2�0�5�0�5
        euler = euler_from_quaternion([q[1], q[2], q[3], q[0]])
        return math.degrees(euler[2])
    except:
        return None

def main():
    rospy.init_node('imu_raw')
    pub = rospy.Publisher('/filtered_yaw', String, queue_size=10)
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        rospy.logerr("Serial init failed: %s", str(e))
        return

    buffer = b''
    rate = rospy.Rate(50)  # 50Hz���0�6�0�7���0�9�0�2�0�0�0�8
    
    while not rospy.is_shutdown():
        # �0�9�0�9�0�6�0�3�0�7�0�3�0�7�0�3�0�8�0�5�0�6�0�6
        try:
            buffer += ser.read(ser.in_waiting or 1)
        except:
            rospy.logwarn("Serial read error")
            continue
        
        # �0�7�0�7�0�8���0�1���0�9�0�4�0�8�0�5�0�6�0�6�㨹
        while True:
            idx = buffer.find(HEADER_BYTE)
            if idx == -1 or len(buffer) < idx + PACKET_SIZE:
                break
            
            packet = buffer[idx:idx+PACKET_SIZE]
            buffer = buffer[idx+PACKET_SIZE:]
            
            yaw = parse_raw_yaw(packet)
            if yaw is not None:
                # Python 2�0�4�0�3�0�6�0�6�0�8�0�2���0�0���0�4�0�7�0�3�0�0�0�9�0�8�0�5�0�3�0�4
                pub.publish("Yaw: {:.3f}".format(yaw))
        
        rate.sleep()

if __name__ == '__main__':
    main()
