#!/usr/bin/env python
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import QR_Frame_calc as QR
import rospy
import random_driving as rd

import actionlib

import tf
from tf.listener import TransformListener
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

#----------------------VARIABLES----------------------------------

#------------------------x,y,x_next,y_next-----------
QR_coordinates = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
QR_coordinates_world = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
finalWord_list = ["0", "0", "0", "0", "0"]
N_old = 0

transform_succesful = False
QR_spotted = False

stop_driving = Twist()
stop_driving.linear.x = 0
stop_driving.angular.z = 0

# ---------------------MOVEMENTS---------------------------
def goto_position(position,frame):
    goal_position = goal_pose(position,frame)
    client.send_goal(goal_position)
    client.wait_for_result()
    rospy.sleep(2)
    print ("arrived!")


def goal_pose(pose,frame):
    if pose == []:
        goal_pose = []
        return goal_pose
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = frame
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def stop():
    print("Stop driving for a second")
    Cmd_vel.publish(stop_driving)
#------------------------CALLBACKS-----------------
def object_position_callback(message):
    global object_camera_position
    object_camera_position = [message.pose.position.x, message.pose.position.y, message.pose.position.z, 1]
    br.sendTransform((message.pose.position.x, message.pose.position.y, message.pose.position.z),
                     (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w),
                     rospy.Time.now(),
                     "qr_code",
                     "camera_optical_link")
def getQR(data):
    if "0" in finalWord_list:
        if data.data:
            x = float(data.data.split("\r\n")[0].split("=")[1])
            y = float(data.data.split("\r\n")[1].split("=")[1])
            x_next = float(data.data.split("\r\n")[2].split("=")[1])
            y_next = float(data.data.split("\r\n")[3].split("=")[1])
            N = int(data.data.split("\r\n")[4].split("=")[1])
            L = data.data.split("\r\n")[5].split("=")[1]
            global QR_spotted
            QR_spotted = True
            finalWord_list[N-1] = L
            
            global N_old
            if N_old is not N:
                N_old = N
                print (x, y, x_next, y_next, N, L, finalWord_list)
    else:
        finalWord_str = ''.join(finalWord_list)
        print ("Mission complete!")
        print ("Final word: " + finalWord_str)
        rospy.signal_shutdown("reason")


#-------------------MAIN------------------------
if __name__ == '__main__':
    print ("Starting script")
    
    rospy.init_node('QR_finder')
    rate = rospy.Rate(60)
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    br = tf.TransformBroadcaster()
    listener = TransformListener()

    Cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/code_message", String, getQR, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, object_position_callback, queue_size=10)
    
    ##Initial random driving:
    print ("Random Driving mode")
    while not transform_succesful:
        rd.random_driving()
        if QR_spotted:
            stop()
            rospy.sleep(1.)
        
        