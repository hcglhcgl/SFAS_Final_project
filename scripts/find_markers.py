#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def retrieveQR(data):
    finalWord = ["0", "0", "0", "0", "0"]
    x = float(data.split("\r\n")[0].split("=")[1])
    y = float(data.split("\r\n")[1].split("=")[1])
    x_next = float(data.split("\r\n")[2].split("=")[1])
    y_next = float(data.split("\r\n")[3].split("=")[1])
    N = int(data.split("\r\n")[4].split("=")[1])
    L = data.split("\r\n")[5].split("=")[1]


def listener():
	rospy.init_node('subscriberNode', anonymous=True)
	rospy.Subscriber('/visp_auto_tracker/code_message', String, sub_cal)
	rospy.spin()

def sub_cal(data):
    if data.data:
        x, y, x_next, y_next, N, L, finalWord = retrieveQR(data.data)

        print x, y, x_next, y_next, N, L, finalWord

        #call function that moves the robot to next QR-code

if __name__ == '__main__':

    listener()

    """
    data = "X=2.35\r\nY=3.24\r\nX_next=5.3\r\nY_next=5.9\r\nN=3\r\nL=M"

    x, y, x_next, y_next, N, L, finalWord = retrieveQR(data.data)

    print x, y, x_next, y_next, N, L, finalWord

    data = "X=2.35\r\nY=3.24\r\nX_next=5.3\r\nY_next=5.9\r\nN=2\r\nL=A"

    x, y, x_next, y_next, N, L, finalWord = retrieveQR(data.data)

    print x, y, x_next, y_next, N, L, finalWord

    data = "X=2.35\r\nY=3.24\r\nX_next=5.3\r\nY_next=5.9\r\nN=1\r\nL=G"

    x, y, x_next, y_next, N, L, finalWord = retrieveQR(data.data)

    print x, y, x_next, y_next, N, L, finalWord
    """
