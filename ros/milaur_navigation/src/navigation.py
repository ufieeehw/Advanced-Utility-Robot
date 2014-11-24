#!/usr/bin/env python

from __future__ import division
## OpenCV
from sensor_msgs.msg import Image
import cv2, cv
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


class Navigator(object):
    def __init__(self):
        rospy.init_node('navigation')
        self.logitech_listener = rospy.Subscriber('milaur_cam/image_raw', Image, self.process_image)
        self.direction_pub = rospy.Publisher('milaur/angle_error', Float64, queue_size=1)

        # create ROS to CV bridge
        self.bridge = CvBridge()

        # define range for pink color in BGR
        self.upper_pink = np.array([180, 255, 255], dtype=np.uint8)
        self.lower_pink = np.array([160, 80, 100], dtype=np.uint8)

        
    def process_image(self, msg):
        # Get raw image from msg
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        # Get the center of the image
        height, width, depth = raw_img.shape
        centX = (width/2)
        centY = (height/2)

        # Blur Image
        blur = cv2.GaussianBlur(raw_img,(5,5),0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # Threshold HSV image for pink
        mask = cv2.inRange(hsv, self.lower_pink, self.upper_pink)

        # Bitwise-AND mask and original image... may not need
        res = cv2.bitwise_and(raw_img, raw_img, mask= mask)

        # Use contours to find object
        contours, hierarchy =cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # If no objects found, just leave
        if len(contours) == 0:
            self.send_target_angle(0)
            return

        # Find the index of the largest contour
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        cnt=contours[max_index]

        # Use bounding box to find most probable center of blob
        x, y, w, h = cv2.boundingRect(cnt)
        blobX = (int)(x+(w/2))
        blobY = (int)(y+(h/2))

        # Angle Math: Focal Length = 3.67mm Pixel Pitch = 3.98um
        pixelPitch = .00398 #.06414
        focalLength = 3.67
        targetAngle = math.atan2((blobX-centX)*pixelPitch, focalLength)

        #print "Target Angle is:", targetAngle

        # debug stuffs
        '''
        cv2.rectangle(raw_img,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.circle(raw_img,(blobX, blobY), 1, (0,255,0), 2)
        cv2.circle(raw_img,(centX, centY), 1, (0,255,0), 2)
        cv2.imshow('Raw', raw_img)
        cv2.imshow('Mask', mask)

        cv2.waitKey(0)
        '''

        self.send_target_angle(targetAngle)


    def send_target_angle(self, angle):
        '''Added to enable ros publishing of angle error'''
        self.direction_pub.publish(Float64(angle))


if __name__ == '__main__':
    navigator_node = Navigator()
    rospy.spin()     


