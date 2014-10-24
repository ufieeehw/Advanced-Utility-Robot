#!/usr/bin/env python
from __future__ import division
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
from sensor_msgs.msg import LaserScan


class Navigator(object):
    _min_angle = np.radians(-270/2)
    _max_angle = np.radians(270/2)
    _avoidance_range = 4

    def __init__(self):
        rospy.init_node('navigation')
        self.lidar_listener = rospy.Subscriber('/scan', LaserScan, self.got_lidar_data)
        self.direction_pub = rospy.Publisher('MILAUR/angle_error', Float64)

        rospy.set_param("/hokuyo_node/min_ang", self._min_angle)
        rospy.set_param("/hokuyo_node/max_ang", self._max_angle)

        # (Lucas/Madison) my own stuff
        self.totalLength = len(ranges)
        self.halfLength = totalLength/2
        self.sumLeft = 0
        self.sumRight = 0
        self.whereToGo = 0
        
    def got_lidar_data(self, msg):
        # Pasted from the init
        lidar_scan = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.ranges = msg.ranges

        if self.check_turn():
            sumLeft, sumRight = self.getSums()
            # Pasted from the turn function
            if sumLeft > sumRight:
                # Turn right
                self.send_target_angle(np.radians(40))
            else:
                # Turn left
                self.send_target_angle(np.radians(-40))
        else:
            # Send a zero if we're okay to go forward
            self.send_target_angle(0)


    def send_target_angle(self, angle):
        '''Added to enable ros publishing of angle error'''
        self.direction_pub.publish(Float64(angle))

    def get_ranges_by_angles(self, min_angle, max_angle, ranges):
        # Index to angle conversion
        index_angle = (self.angle_max - self.angle_min) / self.angle_increment
        start = int(min_angle * (1 / index_angle))
        end = int(max_angle * (1 / index_angle))
        return ranges[start:end - 1]

    def getSums(self):
        self.sumLeft = self.sumRight = 0
        for x in self.ranges[0:self.halfLength]:
            sumLeft = sumLeft + x
        for x in self.ranges[self.halfLength:self.totalLength - 1]:
            sumRight = sumRight + x
        return sumLeft, sumRight

    def check_turn(self):
        # if self.angle_increment > 1.7453 and self.angle_increment < 2.4434: #btw 100 and 140 degrees
            too_close_count = 0
            # Reimplement the above
            for x in self.get_ranges_by_angles(min_angle=100, max_angle=170):
                if x < 1500:
                    too_close_count += 1
            if too_close_count > 5:
                return True
            else:
                return False # Don't turn because we didn't find any obstacles
             
    def find_available_ranges(self, ranges):
        groups = []
        for index, _range in enumerate(ranges):
            if _range > self._avoidance_range:
                end = index

if __name__=='__main__':
    navigator_node = Navigator()
    rospy.spin()​
