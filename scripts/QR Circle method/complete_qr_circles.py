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
finalWord_list = [0, 0, 0, 0, 0]
N_old = 0

transform_succesful = False
QR_spotted = False

stop_driving = Twist()
stop_driving.linear.x = 0
stop_driving.angular.z = 0


first_coordinates_qr = []
first_coordinates_world = []
coordinates_ready_qr = False
coordinates_ready_world = False

turns_spinning = 0

framepoint = 0
angle = 0

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
    print("Stop driving for 3 seconds")
    Cmd_vel.publish(stop_driving)
#------------------------CALLBACKS-----------------
def object_position_callback(message):
    global object_camera_position
    object_camera_position = message
    br.sendTransform((object_camera_position.pose.position.x, object_camera_position.pose.position.y, object_camera_position.pose.position.z),
                     (object_camera_position.pose.orientation.x, object_camera_position.pose.orientation.y, object_camera_position.pose.orientation.z, object_camera_position.pose.orientation.w),
                     rospy.Time.now(),
                     "qr_code",
                     "camera_optical_link")
    
def getQR(data):
    if 0 in finalWord_list:
        if data.data:
            x = float(data.data.split("\r\n")[0].split("=")[1])         # x-coordinate of current QR target
            y = float(data.data.split("\r\n")[1].split("=")[1])         # y-coordinate of current QR target
            x_next = float(data.data.split("\r\n")[2].split("=")[1])    # x-coordinate of next QR target
            y_next = float(data.data.split("\r\n")[3].split("=")[1])    # y-coordinate of next QR target
            N = int(data.data.split("\r\n")[4].split("=")[1])           # ID-number of current QR target
            L = data.data.split("\r\n")[5].split("=")[1]                # Letter from current QR target
            
            if L not in finalWord_list:
                stop()
                finalWord_list[N-1] = L
                print (x, y, x_next, y_next, N, L, finalWord_list)
                
                global QR_spotted
                QR_spotted = True
                
                QR_coordinates[N-1][0] = x
                QR_coordinates[N-1][1] = y
                QR_coordinates[N-1][2] = x_next
                QR_coordinates[N-1][3] = y_next
                
                first_coordinates_qr.extend((x,y))
                if len(first_coordinates_qr) > 3:
                    global coordinates_ready_qr
                    coordinates_ready_qr = True
            
                
    else:
        finalWord_str = ''.join(finalWord_list)
        print ("Mission complete!")
        print ("Final word: " + finalWord_str)
        rospy.signal_shutdown("reason")
        
def printOdo(data):

    (trans, rot) = listener.lookupTransform('/odom', '/qr_code', rospy.Time(0))

    print (trans)
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
    rospy.Subscriber('odom', Odometry, printOdo, queue_size=10)
    ##Initial random driving:
    print ("Random Driving mode")
    while not transform_succesful:
        turns_spinning = turns_spinning + 1
        if turns_spinning > 200000: #200000 seems to be 1 revolution
            rd.random_driving()
        else:
            rd.spinning_around()
        if QR_spotted:
            stop()
            rospy.sleep(3.)
            
            (trans, rot) = listener.lookupTransform('/odom', '/qr_code', rospy.Time(0))
            first_coordinates_world.extend((trans[0], trans[1]))
            
            print(first_coordinates_world)
            
            if len(first_coordinates_world) > 3:
                coordinates_ready_world = True
                print ("World coordinates are ready!")
                
            if coordinates_ready_world and coordinates_ready_qr:
                print (first_coordinates_world[0:4])
                print (first_coordinates_qr[0:4])
                
                framepoint,angle = QR.QR_frame_calc(first_coordinates_world[0:4],first_coordinates_qr[0:4])
                print ("Framepoint: ",framepoint, "Angle: ",angle)
                transform_succesful = True
            QR_spotted = False
    
        
        
