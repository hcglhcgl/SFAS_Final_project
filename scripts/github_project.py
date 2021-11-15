#!/usr/bin/env python

import numpy as np
import rospy
import re # regular expression
import math
import copy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped, Twist

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf
from tf.listener import TransformListener
from tf.transformations import quaternion_from_euler

from nav_msgs.msg import Odometry


global qr_code_info # secret frame coordinates and qr code tag read from the topic '/visp_auto_tracker/code_message'; ["", "", "", "", ""]
qr_code_info = []

global objects_secret_position # secret frame coordinates of all qr code markers; [[, , 0, 1], [, , 0, 1]]
objects_secret_position = []
global object_secret_position # secret frame coordinates of current qr code marker read from the topic '/visp_auto_tracker/object_position'; [, , 0, 1]
object_secret_position = []

global objects_odom_position # odom frame coordinates of all qr code markers; [[, , 0, 1], [, , 0, 1]]
objects_odom_position = []
global object_odom_position # odom frame coordinates of current qr code marker; [, , 0, 1]
object_odom_position = []

global next_object_secret_position # secret frame coordinates of next qr code marker read from the topic '/visp_auto_tracker/object_position'; [, , 0, 1]
next_object_secret_position = []

global SMaRt_element # last character read from the topic '/visp_auto_tracker/code_message'; ''
SMaRt_element = ''
global SMaRt_messages # all SMart_element; '{'1': 'S', '2': 'M', '3': 'a', '4': 'R', '5': 't'}'
SMaRt_messages = {}


global tags # all tag; ['', '', '', '', '']
tags = ['0']
global tag # fifth number read from the topic '/visp_auto_tracker/code_message'; ''
tag = '0'

def goal_pose(pose):
    if pose == []:
        goal_pose = []
        return goal_pose
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def trans_secret_odom(objects_odom_position, objects_secret_position):
    #print(objects_odom_position)
    #print(objects_secret_position)
    
    # A = np.array([[float(objects_secret_position[0][0]), -float(objects_secret_position[0][1]), 1, 0],
    #         [float(objects_secret_position[0][1]), float(objects_secret_position[0][0]), 0, 1],
    #         [float(objects_secret_position[1][0]), -float(objects_secret_position[1][1]), 1, 0],
    #         [float(objects_secret_position[1][1]), float(objects_secret_position[1][0]), 0, 1]])

    # b = np.array([[objects_odom_position[0][0], objects_odom_position[0][1], objects_odom_position[1][0], objects_odom_position[1][1]]]).T

    # results = np.linalg.solve(A, b)
    
    sub_x_world_12 = float(objects_odom_position[0][0]) - float(objects_odom_position[1][0])
    sub_y_world_12 = float(objects_odom_position[0][1]) - float(objects_odom_position[1][1])
    sub_x_secret_12 = float(objects_secret_position[0][0]) - float(objects_secret_position[1][0])
    sub_y_secret_12 = float(objects_secret_position[0][1]) - float(objects_secret_position[1][1])

    sin_theta = (sub_x_secret_12 * sub_y_world_12 - sub_y_secret_12 * sub_x_world_12) / (pow(sub_x_secret_12, 2) + pow(sub_y_secret_12, 2))
    cos_theta = (sub_x_world_12 + sin_theta * sub_y_secret_12) / sub_x_secret_12
    theta = math.atan2(sin_theta, cos_theta)
    x_ref_world = float(objects_odom_position[1][0]) - cos_theta * float(objects_secret_position[1][0]) + sin_theta * float(objects_secret_position[1][1])
    y_ref_world = float(objects_odom_position[1][1]) - sin_theta * float(objects_secret_position[1][0])- cos_theta * float(objects_secret_position[1][1])
    results = [cos_theta, sin_theta, x_ref_world, y_ref_world]

    T = np.array([[results[0], -results[1], 0, results[2]], 
            [results[1], results[0], 0, results[3]], 
            [0, 0, 1, 0],
            [0, 0, 0, 1]], dtype='float64')
    return T
 
def eulerAnglesToRotationMatrix(r, p, y):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r), math.cos(r)]])

    R_y = np.array([[math.cos(p), 0, math.sin(p)],
                    [0, 1, 0],
                    [-math.sin(p), 0, math.cos(p)]])

    R_z = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y), math.cos(y), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def object_position_callback(message):
    global object_camera_position
    object_camera_position = [message.pose.position.x, message.pose.position.y, message.pose.position.z, 1]
    br.sendTransform((message.pose.position.x, message.pose.position.y, message.pose.position.z),
                     (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w),
                     rospy.Time.now(),
                     "qr_code",
                     "camera_optical_link")
            
def code_message_callback(message):
    global qr_code_info
    global object_secret_position
    global next_object_secret_position
    global tag
    global SMaRt_element
    qr_code_info = re.findall(r"[-+]?\d*\.\d+|\d+", str(message))
    if qr_code_info != []:
        object_secret_position = [qr_code_info[0], qr_code_info[1], 0, 1]
        next_object_secret_position = [qr_code_info[2], qr_code_info[3], 0, 1]
        tag = qr_code_info[4]
        SMaRt_element = str(message)[-2]


def goto_observe_position(observe_position):
    goal_position = goal_pose(observe_position)
    client.send_goal(goal_position)
    client.wait_for_result()
    rospy.sleep(2)
    print "arrived!" 

if __name__ == '__main__':
    rospy.init_node('patrol')
    rate = rospy.Rate(10)
    print "Starting script"
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    br = tf.TransformBroadcaster()
    listener = TransformListener()

    Cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/code_message", String, code_message_callback, queue_size=10)
    rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, object_position_callback, queue_size=10)

    Trans_secret_odom = []
    # go to the observation position
    observe_pose = [[[-4.5, 0, 0], [0, 0, 0, 1]], [[6, 0, 0], [0, 0, 0, 1]], [[-4.5, 2, 0], [0, 0, 0, 1]], [[-4.5, -2, 0], [0, 0, 0, 1]], [[6, -2, 0], [0, 0, 0, 1]], [[6, 2, 0], [0, 0, 0, 1]]]
    for observation_point in observe_pose:
        if Trans_secret_odom != []:
            break
        print "Go to the observation position", "(", observation_point[0][0], ", ", observation_point[0][1], ")"
        goto_observe_position(observation_point)
        print "=================================================="
        print "Searching the first two qr code markers..."
        initial_time = rospy.Time.now()
        while(np.size(tags)<3):
            if rospy.Time.now() - initial_time >= rospy.Duration(secs=25):
                print "We should go to another observation point..."
                break
            object_odom_position_solved = False
            twist_1 = Twist()
            twist_1.angular.z = 0.3
            Cmd_vel.publish(twist_1)
            while not object_odom_position_solved:
                if tag not in tags:
                    print("Find tag:",tag)
                    tags.append(tag)
                    twist_2 = Twist()
                    Cmd_vel.publish(twist_2)
                    rospy.sleep(1)
                    SMaRt_messages.update({int(tag): SMaRt_element})
                    try:
                        # obtatin object_odom_position
                        #(trans, rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
                        (trans, rot) = listener.lookupTransform('/odom', '/qr_code', rospy.Time(0))
                        # (r_cam, p_cam, y_cam) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
                        # trans_rotation_cam_odom = np.insert(eulerAnglesToRotationMatrix(r_cam, p_cam, y_cam), 3, [0, 0, 0], 0)
                        # trans_translation_cam_odom = [trans[0], trans[1], trans[2], 1]
                        # trans_cam_odom = np.insert(trans_rotation_cam_odom, 3, trans_translation_cam_odom, 1)

                        # object_odom_position = np.dot(trans_cam_odom, object_camera_position)
                        # objects_odom_position.append(object_odom_position)
                        objects_odom_position.append([trans[0], trans[1], 0, 1])
                        objects_secret_position.append(object_secret_position)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                object_odom_position_solved = True
                # print("object_odom_position: ", object_odom_position)
            if np.size(tags)==3:
                # print("objects_odom_position: ", objects_odom_position)
                # print("objects_secret_position: ", objects_secret_position)
                Trans_secret_odom = trans_secret_odom(objects_odom_position, objects_secret_position)
                print "=================================================="
                print "The transform from secret frame to odom frame is: "
                print Trans_secret_odom
                print "=================================================="

    while object_odom_position_solved:
        if len(SMaRt_messages) == 5:
            break
        goal_tag = copy.deepcopy(tag)
        print "=================================================="
        print "Current tag: ", tag 
        next_robot_secret_position = [float(next_object_secret_position[0])*1, float(next_object_secret_position[1])*1]
        next_robot_odom_position = np.dot(Trans_secret_odom, np.insert(next_robot_secret_position, 2, [0, 1], 0))
        next_robot_odom_position[0] = next_robot_odom_position[0] * 0.65
        next_robot_odom_position[1] = next_robot_odom_position[1] * 0.65
        next_robot_odom_position = [next_robot_odom_position, [0,0,0,1]]

        print "next_robot_odom_position: "
        print next_robot_odom_position
        goal_position = goal_pose(next_robot_odom_position)
        client.send_goal(goal_position)
        client.wait_for_result()
        rospy.sleep(5)
        if int(tag) - int(goal_tag) == 1 or int(tag) - int(goal_tag) == -4:
            twist_3 = Twist()
            Cmd_vel.publish(twist_3)
            rospy.sleep(1)
            SMaRt_messages.update({int(tag): SMaRt_element})
            tags.append(tag)
            continue
        else:
            while(int(tag) - int(goal_tag) != 1 and int(tag) - int(goal_tag) != -4):
                twist_3 = Twist()
                twist_3.angular.z = 0.3
                Cmd_vel.publish(twist_3)
            twist_4 = Twist()
            Cmd_vel.publish(twist_4)
            rospy.sleep(1)
            SMaRt_messages.update({int(tag): SMaRt_element})
            tags.append(tag)
            continue
        
    SMaRt_messages = sorted(SMaRt_messages.items(), key=lambda x:x[0])
    print "=================================================="
    print SMaRt_messages