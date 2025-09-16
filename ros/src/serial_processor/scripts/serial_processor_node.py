#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import serial
import numpy as np
import re
import time
from std_msgs.msg import Float32MultiArray, String

class SerialProcessor:
    def __init__(self):
        rospy.init_node('serial_processor_node', anonymous=True)

        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        

        self.pub_dist = rospy.Publisher('/distance_data', Float32MultiArray, queue_size=10)
        self.pub_xy = rospy.Publisher('/xy_coordinates', Float32MultiArray, queue_size=10)
        self.pub_forward = rospy.Publisher('/forwarded_data', String, queue_size=10)
        

        self.ser = self._init_serial()
        if not self.ser:
            rospy.logerr("Serial initialization failed")
            exit(1)
            

        self.anchor_coords = {
            0: (0.0, 0.0),   # A0
            1: (1.0, 0.0),     # A1 
            2: (0.0, 1.0),     # A2
            3: (1.0, 0.0)      # A3
        }
        
        self.last_valid_data = None
        self.last_log_time = time.time()
        
        self.main_loop()

    def _init_serial(self):
        """8N1�0�7�0�3�0�7�0�3�0�6�0�1�0�8�0�4�0�3�0�4"""
        try:
            return serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
        except Exception as e:
            rospy.logerr("Serial init error: {}".format(str(e)))
            return None

    def _parse_mc_packet(self, raw):
        """�0�5�0�9�0�2�0�2mc�0�4�0�2�0�6���0�8�0�5�0�6�0�6�㨹"""
        pattern = r"mc (\w{2}) (\w{8}) (\w{8}) (\w{8}) (\w{8})"
        match = re.match(pattern, raw.strip())
        if not match:
            return None
            
        mask = int(match.group(1), 16)
        ranges = []
        for i in range(4):
            hex_val = match.group(i+2)
            if (mask >> i) & 0x01:  # �0�4���0�5��mask�0�7�0�4�0�4��0�2�0�3
                ranges.append(int(hex_val, 16) / 1000.0)  # mm���0�9�0�1��
            else:
                ranges.append(None)
        return ranges

    def _parse_kt0_packet(self, raw):
        """�0�5�0�9�0�2�0�2$KT0�0�4�0�2�0�6���0�8�0�5�0�6�0�6�㨹"""
        if not raw.startswith("$KT0"):
            return None
            
        parts = [p.strip() for p in raw.split(',')]
        if len(parts) < 6:
            return None
            
        ranges = []
        for p in parts[1:5]:  # �0�5��4�0�0�0�2�0�2�0�9�0�6���0�8�0�5�0�0�0�8
            if p == 'NULL':
                ranges.append(None)
            else:
                try:
                    ranges.append(float(p))
                except:
                    ranges.append(None)
        return ranges

    def _trilaterate(self, distances):
        """�0�6�0�5���0�8�0�9���0�2�0�3�0�2�0�9�0�4�0�2�0�9�0�0����"""
        valid_data = [(d, idx) for idx, d in enumerate(distances) if d is not None]
        if len(valid_data) < 3:
            return None, None
            
        # �0�5�0�3�0�8�0�9���0�6�0�5���0�8�0�23�0�0�0�2�0�1�0�9�0�8�0�0
        valid_data.sort(key=lambda x: x[0])
        used_anchors = valid_data[:3]
        
        A = []
        B = []
        try:

            (x0, y0) = self.anchor_coords[used_anchors[0][1]]
            d0 = used_anchors[0][0]
            
            for d, idx in used_anchors[1:3]:
                xi, yi = self.anchor_coords[idx]
                # �0�1�0�1�0�5���0�6�0�1�0�9�����0�5�0�6�0�0
                A.append([2*(xi - x0), 2*(yi - y0)])
                B.append(d0**2 - d**2 + xi**2 - x0**2 + yi**2 - y0**2)
                
            A_np = np.array(A)
            B_np = np.array(B)
            X = np.linalg.lstsq(A_np, B_np, rcond=None)[0]
            return X[0], X[1]
        except Exception as e:
            rospy.logwarn("Trilateration error: {}".format(str(e)))
            return None, None

    def _publish_results(self, distances, x, y):
        """�0�1�0�6�0�6�0�3�0�8�0�5�0�6�0�6���0�4�0�5�0�4���0�5����"""
        # ���0�4�0�5�0�4�0�8�0�2�0�8�0�4�0�6���0�8�0�5�0�8�0�5�0�6�0�6�0�5���0�2�0�7�0�4��0�0�0�8�0�7���0�2�0�9-1�0�5�0�8
        dist_msg = Float32MultiArray()
        dist_msg.data = [d if d is not None else -1.0 for d in distances]
        self.pub_dist.publish(dist_msg)
        
        # ���0�4�0�5�0�4���0�3�����0�8�0�5�0�6�0�6
        if x is not None and y is not None:
            xy_msg = Float32MultiArray()
            xy_msg.data = [x, y]
            self.pub_xy.publish(xy_msg)
            
            # �0�0�0�9�0�8�0�5�0�3�0�4���0�9���0�4�0�8�0�5�0�6�0�6
            forward_msg = String()
            forward_msg.data = "POSITION: {:.2f}, {:.2f}".format(x, y)
            self.pub_forward.publish(forward_msg)
            
            # �0�3�0�7�0�9�0�2�0�6�0�9�0�0�0�6�0�8�0�1�0�6�0�2
            if time.time() - self.last_log_time > 1.0:
                rospy.loginfo("�0�9���0�2�0�3���0�3����: X={:.2f}m, Y={:.2f}m".format(x, y))
                self.last_log_time = time.time()

    def main_loop(self):
        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            try:
                # �0�9�0�9�0�6�0�3�0�1���0�9�0�4�0�8�0�5�0�6�0�6�㨹
                raw = self.ser.read_until(b'\r\n').decode('ascii', errors='ignore').strip()
                if not raw:
                    continue
                
                # �0�4�0�2�0�6�����0�0���0�4�0�7�0�7�0�8��
                ranges = None
                if raw.startswith("mc"):
                    ranges = self._parse_mc_packet(raw)
                elif raw.startswith("$KT0"):
                    ranges = self._parse_kt0_packet(raw)
                
                if ranges:
                    x, y = self._trilaterate(ranges)
                    self._publish_results(ranges, x, y)
                    
            except UnicodeDecodeError:
                rospy.logwarn("Serial data decode error")
            except Exception as e:
                rospy.logerr("Main loop error: {}".format(str(e)))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        SerialProcessor()
    except rospy.ROSInterruptException:
        pass
