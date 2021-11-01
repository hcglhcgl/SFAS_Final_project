#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def sub_cal(data):
    rospy.loginfo(data.data)


def listener():
	rospy.init_node('subscriberNode', anonymous=True)
	rospy.Subscriber('/visp_auto_tracker/code_message', String, sub_cal)
	rospy.spin()

if __name__ == '__main__':
	listener()
