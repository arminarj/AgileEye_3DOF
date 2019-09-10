#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import os
import imutils
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from imutils.video import VideoStream
from collections import deque
from dynamixel_sdk import *
import cv2

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import config
from agile_eye import AGILE
# from agile_eye import EYE

TORQUE_DISABLE              = 0
READ_POSITION               = 0
READ_VELOCITY               = 1

# def wavemake(ampl, freq, phas, time, ticks):
#     a1 = ampl[0]; a2 = ampl[1]; a3 = ampl[2]
#     f1 = freq[0]; f2 = freq[1]; f3 = freq[2]
#     p1 = phas[0]; p2 = phas[1]; p3 = phas[2]
#     t = np.linspace(0, time, ticks)
#     v1 = a1 * f1 * np.cos(f1 * t - p1)
#     v2 = a2 * f2 * np.cos(f2 * t - p2)
#     v3 = a3 * f3 * np.cos(f3 * t - p3)
#     return np.int32(np.transpose([v1, v2, v3]))


# gazer = AGILE('/dev/ttyUSB0', warnMe='off')
# motion = wavemake([20, 20, 20], [4, 1, 9], [0, 0, 0], 10, 150)

# gazer.openPort()
# gazer.setBaudRate()
# gazer.home()
# gazer.wait_for_key()
# gazer.initiate()

# P  = 0.1
# theta_1 = -46.1568378349 * P
# theta_2 = 72.3181345519 * P
# theta_3 = 20.1674230165 * P


# temp_ = np.transpose([int(theta_1),int(theta_2),int(theta_3)])

# # # list1, list2 = [], []
# # for item in motion:
# #     gazer.move_with_velocity(item)
# #     # list1.append(gazer.read_from_motor(READ_POSITION)[1])
# #     # list2.append(item[1])

# gazer.move_with_velocity(temp_)
# sleep(1)
# gazer.home()

# gazer.wait_for_key()



gazer = AGILE('/dev/ttyUSB0', warnMe='off')


def Agile_initial():
    global gazer
    gazer.openPort()
    gazer.setBaudRate()
    gazer.home()
    gazer.wait_for_key()
    gazer.initiate()

def saturate(theta_1,theta_2,theta_3):
    MARGIN = 6
    theta_1_ = min(theta_1, MARGIN)
    theta_2_ = min(theta_2, MARGIN)
    theta_3_ = min(theta_3, MARGIN)
    return (theta_1_,theta_2_,theta_3_)

def PID_Controller(theta_1,theta_2,theta_3):
    P  = 0.3
    theta_1_ = theta_1 * P
    theta_2_ = theta_2 * P
    theta_3_ = theta_3 * P
    (theta_1,theta_2,theta_3) = saturate(theta_1,theta_2,theta_3)
    return (theta_1_,theta_2_,theta_3_)


def move_vel(theta_1,theta_2,theta_3):
    (theta_1,theta_2,theta_3) = PID_Controller(theta_1,theta_2,theta_3)
    new_move = np.int32(np.transpose([float(theta_1),float(theta_2),float(theta_3)]))
    # new_move = np.transpose([int(5),int(5),int(5)])
    print("new_move :", new_move)
    gazer.move_with_velocity(new_move)

zero = 0

def callback(req):
    
    try:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", req.data)
        
        # global theta_1, theta_2, theta_3
        data = req.data.split(',')
        print(data)
        theta_1 =  np.int32(float(data[0]) + 17 )
        theta_2 =  np.int32(float(data[1]) - 25 )
        theta_3 =  np.int32(float(data[2]) - 80 )
        new_move = np.array([theta_1, theta_2, theta_3])

        # read_deg = np.abs(gazer.read_from_motor(READ_POSITION))
        # print( "Position : " , read_deg )
        if ( np.max(new_move) >= 10 ):
            move_vel(zero,zero,zero)
            return
        
        move_vel(theta_1,theta_2,theta_3)
        
        # sleep(0.01)
        # move_vel(zero,zero,zero)
        
    except Exception ,e: 
        print(e)
        print('exception catched.')
        pass
    
        
    
def listener():
    global gazer
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('DYN_node', anonymous=False)
    rospy.Subscriber("Motors_theta", String, callback)
    #  pub = rospy.Publisher('topic1', String, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    global gazer
    # gazer = AGILE('/dev/ttyUSB0', warnMe='off')
    Agile_initial()
    listener()