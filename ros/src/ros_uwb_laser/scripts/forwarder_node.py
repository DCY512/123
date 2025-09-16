#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import re  # �0�8�0�4�0�6�0�5�0�9�0�5�0�8�������0�7�0�7�0�8�0�5�0�2�0�5�0�7��
from std_msgs.msg import String

def forwarded_data_callback(msg):
    """ �0�7�0�7�0�8���0�5�0�7�0�8�0�9�0�8�0�5�0�8�0�2�0�8�0�5�0�6�0�6�0�5�0�1�0�5�0�4�0�0���0�6�0�3�0�8�0�5�0�0�0�8 """
    rospy.loginfo("Raw received data: {}".format(msg.data))  # �0�7���0�7�0�3�0�5�0�7�0�8�0�9�0�8�0�5�0�8�0�2�0�8�0�2�0�8�0�4�0�8�0�5�0�6�0�6

    # �0�8�0�1�0�7�0�1�0�9�0�5�0�8�������0�7�0�7�0�8�0�5�0�0���0�6�0�3�0�9���0�7�0�4�0�8�0�2�0�4���0�0�0�8�0�9�0�8
    matches = re.findall(r'"(\w+)":(\d+)', msg.data)  # �0�4�0�6�0�3�0�1�0�0�0�9�0�8�0�5 "key":value

    if matches:
        # �0�7�0�7�0�5���0�6�0�3�0�0�0�2���0�0�0�8�0�1�0�8�0�7�0�7�0�3�0�7�0�4�0�0���0�6�0�3�0�8�0�2�0�4���0�0�0�8�0�9�0�8
        extracted_data = {key: int(value) for key, value in matches}
        rospy.loginfo("Extracted data: {}".format(extracted_data))  # �0�7���0�7�0�3�0�0���0�6�0�3�0�8�0�2�0�8�0�5�0�0�0�8�0�8�0�5�0�6�0�6
    else:
        rospy.logerr("No key-value pairs found in the received data")

def forwarder_node():
    """ ROS �0�5�0�3�0�8�0�0�0�0�0�2�0�4�0�8�0�5 """
    rospy.init_node('forwarder_node', anonymous=True)
    rospy.loginfo("Forwarder Node started")
    
    # �0�9�0�8�0�8�0�2 /forwarded_data �0�3��0�0�0�9
    rospy.Subscriber('/forwarded_data', String, forwarded_data_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        forwarder_node()
    except rospy.ROSInterruptException:
        pass

