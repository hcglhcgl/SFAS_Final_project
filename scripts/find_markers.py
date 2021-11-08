#!/usr/bin/env python

import rospy
from std_msgs.msg import String

finalWord_list = ["0", "0", "0", "0", "0"]
N_old = 0

def QRlistener():
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber('/visp_auto_tracker/code_message', String, retrieveQR)
    rospy.spin()

def retrieveQR(data):
    if "0" in finalWord_list:
        if data.data:
            x = float(data.data.split("\r\n")[0].split("=")[1])
            y = float(data.data.split("\r\n")[1].split("=")[1])
            x_next = float(data.data.split("\r\n")[2].split("=")[1])
            y_next = float(data.data.split("\r\n")[3].split("=")[1])
            N = int(data.data.split("\r\n")[4].split("=")[1])
            L = data.data.split("\r\n")[5].split("=")[1]

            finalWord_list[N-1] = L
            
            global N_old
            if N_old is not N:
                N_old = N
                print x, y, x_next, y_next, N, L, finalWord_list

            #call function that moves the robot to next QR-code
    else:
        finalWord_str = ''.join(finalWord_list)
        print "Done with exploring"
        print "Final word: " + finalWord_str
        rospy.signal_shutdown("reason") 

if __name__ == '__main__':
    QRlistener()