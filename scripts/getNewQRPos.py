#!/usr/bin/env python

import numpy as np
from numpy import * 



def getNewQRPos(x, y, x_next, y_next):

    turtle_pos_x = 3
    turtle_pos_y = 0.5
    turtle_pos_z = 0

    turtle_to_qr1_z = 0
    turtle_to_qr1_x = 1.5
    turtle_to_qr1_y = 0

    turtle_pos = array([[1, 0, 0, turtle_pos_x],
                            [0, 1, 0, turtle_pos_y],
                            [0, 0, 1, turtle_pos_z],
                            [0, 0, 0, 1]])

    turtle_to_qr1 = array([[1, 0, 0, turtle_to_qr1_z],
                                [0, 1, 0, turtle_to_qr1_x],
                                [0, 0, 1, turtle_to_qr1_y],
                                [0, 0, 0, 1]])

    robot_to_qr1 = turtle_to_qr1 + turtle_pos


    qr1_z = x
    qr1_x = y
    qr1_y = 0

    qr1_position = array([[1, 0, 0, qr1_z],
                        [0, 1, 0, qr1_x],
                        [0, 0, 1, qr1_y],
                        [0, 0, 0, 1]])

    qr2_z = x_next
    qr2_x = y_next
    qr2_y = 0

    qr2_position = array([[1, 0, 0, qr2_z],
                        [0, 1, 0, qr2_x],
                        [0, 0, 1, qr2_y],
                        [0, 0, 0, 1]])

    between_qr1_qr2 = qr1_position - qr2_position

    robot_to_qr2 = robot_to_qr1 - between_qr1_qr2

    print robot_to_qr2


if __name__ == '__main__':
    print "Starting program"

    x = 1
    y = 1
    x_next = -0.5
    y_next = 1

    getNewQRPos(x, y, x_next, y_next)