#!/usr/bin/env python

import rospy
from std_msgs.msg import String

finalWord = ["0", "0", "0", "0", "0"]
def retrieveQR(data):
    
    x = float(data.split("\r\n")[0].split("=")[1])
    y = float(data.split("\r\n")[1].split("=")[1])
    x_next = float(data.split("\r\n")[2].split("=")[1])
    y_next = float(data.split("\r\n")[3].split("=")[1])
    N = int(data.split("\r\n")[4].split("=")[1])
    L = data.split("\r\n")[5].split("=")[1]

    finalWord[N-1] = L
    finalWord = ''.join(finalWord)

    return x, y, x_next, y_next, N, L


def listener():
	rospy.init_node('subscriberNode', anonymous=True)
	rospy.Subscriber('/visp_auto_tracker/code_message', String, sub_cal)
	rospy.spin()

def sub_cal(data):
    if data.data:
        x, y, x_next, y_next, N, L = retrieveQR(data.data)

        print x, y, x_next, y_next, N, L, finalWord

        #call function that moves the robot to next QR-code

if __name__ == '__main__':
    listener()