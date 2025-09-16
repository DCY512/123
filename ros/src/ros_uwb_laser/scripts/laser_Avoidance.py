#!/usr/bin/env python2
# coding:utf-8

import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserAvoidPIDConfig
from std_msgs.msg import Float32MultiArray  # 修改为 MultiArray 以接收 xy 数据
import json
import re
import time

RAD2DEG = 180 / math.pi

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.Moving = False
        self.switch = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        # 初始化 DCYX, DCYY, HUO 状态
        self.dcx_state = {"DCYX": 0, "DCYY": 0, "HUO": 0}
        self.yaw = 0.0  # 此处保留 yaw，如果需要
        self.rotation_active = False
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        self.linear = 0.5
        self.angular = 1.0
        self.ResponseDist = 0.55
        self.LaserAngle = 30

        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        # 修改订阅器类型为 Float32MultiArray，以接收 xy 数据
        self.sub_forwarded = rospy.Subscriber('/forwarded_data', Float32MultiArray, self.forwarded_data_callback)

    def cancel(self):
        if hasattr(self, 'sub_laser'):
            self.sub_laser.unregister()
        if hasattr(self, 'sub_forwarded'):
            self.sub_forwarded.unregister()
        rospy.loginfo("Shutdown complete")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan) or self.switch:
            return

        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # 根据角度范围判断障碍物警告
            if 160 > angle > 180 - self.LaserAngle:
                if ranges[i] < self.ResponseDist:
                    self.Right_warning += 1
            if -160 < angle < self.LaserAngle - 180:
                if ranges[i] < self.ResponseDist:
                    self.Left_warning += 1
            if abs(angle) > 160:
                if ranges[i] <= self.ResponseDist:
                    self.front_warning += 1

        rospy.loginfo("Front: {}, Left: {}, Right: {}".format(
            self.front_warning, self.Left_warning, self.Right_warning))
        rospy.loginfo("DCYX: {}, DCYY: {}, HUO: {}, Yaw: {:.3f}".format(
            self.dcx_state["DCYX"], self.dcx_state["DCYY"], self.dcx_state["HUO"], self.yaw))

    def forwarded_data_callback(self, msg):
        rospy.loginfo("Received forwarded data: %s" % msg.data)
        # 假设 msg.data 是一个数组，包含 [x, y]
        if len(msg.data) >= 2:
            self.dcx_state["DCYX"] = msg.data[0]
            self.dcx_state["DCYY"] = msg.data[1]
            rospy.loginfo("Updated DCYX: %.2f, DCYY: %.2f" % (self.dcx_state["DCYX"], self.dcx_state["DCYY"]))
        else:
            rospy.loginfo("Forwarded data length < 2: %s" % msg.data)

if __name__ == '__main__':
    rospy.init_node('laser_Avoidance', anonymous=False)
    tracker = laserAvoid()
    rospy.spin()
