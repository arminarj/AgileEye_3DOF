#!/usr/bin/env python

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String

from sensor_msgs.msg import CameraInfo

# OpenCV2 for saving an image
import cv2
import time
import os
from os.path import expanduser
import datetime
import imutils
import numpy as np
from collections import deque
import argparse




# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
colorLower = (10, 113, 139)
colorUpper = (27, 255, 255)
pts = deque(maxlen=1000)


# Instantiate CvBridge
bridge = CvBridge()

timer = 0

def talker(x,y):
    pub = rospy.Publisher('point_data', String, queue_size=20)
    if not rospy.is_shutdown():
        point_xy = str(x) +','+ str(y) 
        pub.publish(point_xy)
    # print ( "new (%i,%i) published ",str(x), str(y) )


def image_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heared a Frame")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2_img = cv2.flip(cv2_img,1)
        # cv2.imshow('Frame', cv2_img)

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(cv2_img, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # # find the largest contour in the mask, then use
            # # it to compute the minimum enclosing circle and
            # # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            try:
                talker(x,y)
            except ValueError:

                print("Failed.")
            # rospy.message_pub = rospy.Publisher("/center", (int, int), queue_size=10)
            
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # FPS add
        global timer
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        timer = cv2.getTickCount()
        # print ((int(fps)))
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)




        # Save your OpenCV2 image as a jpeg
        cv2.imshow('camera_frame', frame)
        # cv2.imshow('camera_mask', mask)
        # cv2.imshow("camera_hsv", hsv)
        cv2.waitKey(5)
        # cv2.imshow('camera_', cv2_img)
        # key = cv2.waitKey(1)

            

    except  e:
        print(e)


def main():
    global data_writer

    rospy.init_node('usb_cam_listener')
    # Define your image topic
    image_topic = "/usb_cam/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()
    try:
        data_writer.release()
    except: pass

if __name__ == '__main__':
    main()
