#!/usr/bin/env python

import rospy
#import random_driving
from std_msgs.msg import String
from geometry_msgs.msg import Twist

finalWord_list = ["0", "0", "0", "0", "0"]
N_old = 0
gotoNextQR = False

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()


def QRlistener():
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber('/visp_auto_tracker/code_message', String, getQR)
    rospy.spin()

def objectPos():
    print "here1"
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber('/visp_auto_tracker/object_position', String, getObjectPos)
    rospy.spin()

def getObjectPos(data):
    print "here2"
    rospy.loginfo(data.data)

def spinRobot():
    print "You spin me right round"

    twist.angular.z = 0.4

    cmd_vel_pub.publish(twist)

def stopRobot():
    print "Stopping robot"

    twist.linear.x = 0.0
    twist.angular.z = 0.0

    cmd_vel_pub.publish(twist)

def getQR(data):
    global gotoNextQR
    if not gotoNextQR:
        spinRobot()
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

                print "call function that moves the robot to next QR-code"
                stopRobot()
                objectPos()
                gotoNextQR = True
    else:
        finalWord_str = ''.join(finalWord_list)
        print "Done with exploring"
        print "Final word: " + finalWord_str
        rospy.signal_shutdown("reason")


if __name__ == '__main__':
    print "Starting program"
    rospy.init_node('subscriberNode', anonymous=True)
    rate = rospy.Rate(60)
    QRlistener()
    while not rospy.is_shutdown():
       # wander()
       rate.sleep()
