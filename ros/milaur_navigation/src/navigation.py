#!/usr/bin/env python
from __future__ import division
## OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError
## Math
import numpy as np
import math
## Ros/tools
import tf
from tf import transformations as tf_trans
import rospy
## Ros msgs
from std_msgs.msg import Header, Int16, Bool, String, Float64
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import Image


class Navigator(object):
    def __init__(self):
        rospy.init_node('navigation')
        self.logitech_listener = rospy.Subscriber('milaur/image_raw', Image, self.process_image)
        self.direction_pub = rospy.Publisher('milaur/angle_error', Float64)

        # create ROS to CV bridge
        self.bridge = CvBridge()

	# define range for pink color in BGR
        self.lower_pink = np.array([190, 29, 220], dtype=np.uint8)
        self.uopoper_pink = np.array([220, 10, 250], dtype=np.uint8)

        cv2.namedWindow("Image Window", 1)

        
    def process_image(self, msg):
        # Get raw image from msg
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert BGR to HSV
        hsv = cv2.cvtVolot(raw_img, cv2.COLOR_BGR2HSV)

        # Threshold HSV image for pink
        mask = cv2.inRange(hsv, self.lower_pink, self.upper_pink)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(raw_img, raw_img, mask= mask)

        cv2.imshow('Image Window', raw_img)
        #cv2.imshow('mask', mask)
        #cv2.imshow('res', res)

        #if len(groups) != 0:
        #    self.send_target_angle(min(groups, key = lambda x: abs(x)))
        #else:
        #    self.send_target_angle(-15)


    def send_target_angle(self, angle):
        '''Added to enable ros publishing of angle error'''
        self.direction_pub.publish(Float64(angle))


if __name__ == '__main__':
    navigator_node = Navigator()
    rospy.spin()     


