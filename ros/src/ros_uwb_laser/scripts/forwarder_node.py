#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import re  # µ¼ÈëÕýÔò±í´ïÊ½Ä£¿é
from std_msgs.msg import String

def forwarded_data_callback(msg):
    """ ´¦Àí½ÓÊÕµ½µÄÊý¾Ý£¬²¢ÌáÈ¡ÊýÖµ """
    rospy.loginfo("Raw received data: {}".format(msg.data))  # ´òÓ¡½ÓÊÕµ½µÄÔ­Ê¼Êý¾Ý

    # Ê¹ÓÃÕýÔò±í´ïÊ½ÌáÈ¡ËùÓÐµÄ¼üÖµ¶Ô
    matches = re.findall(r'"(\w+)":(\d+)', msg.data)  # Æ¥Åä¸ñÊ½ "key":value

    if matches:
        # ´´½¨Ò»¸ö×ÖµäÀ´´æ´¢ÌáÈ¡µÄ¼üÖµ¶Ô
        extracted_data = {key: int(value) for key, value in matches}
        rospy.loginfo("Extracted data: {}".format(extracted_data))  # ´òÓ¡ÌáÈ¡µÄÊýÖµÊý¾Ý
    else:
        rospy.logerr("No key-value pairs found in the received data")

def forwarder_node():
    """ ROS ½ÚµãÖ÷º¯Êý """
    rospy.init_node('forwarder_node', anonymous=True)
    rospy.loginfo("Forwarder Node started")
    
    # ¶©ÔÄ /forwarded_data »°Ìâ
    rospy.Subscriber('/forwarded_data', String, forwarded_data_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        forwarder_node()
    except rospy.ROSInterruptException:
        pass

