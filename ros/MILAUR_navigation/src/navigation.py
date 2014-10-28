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
    _angular_resolution = np.radians(0.351625)
    _avoidance_range = 2

    _min_clearance = 1.5

    def __init__(self):
        rospy.init_node('navigation')
        self.lidar_listener = rospy.Subscriber('/scan', LaserScan, self.analyze_scan)
        self.direction_pub = rospy.Publisher('MILAUR/angle_error', Float64)

        #rospy.set_param("/hokuyo_node/min_ang", Float64(self._min_angle))
        #rospy.set_param("/hokuyo_node/max_ang", Float64(self._max_angle))
        
    def analyze_scan(self, msg):
        # Pasted from the init
        lidar_scan = msg.ranges

        groups = self.find_available_ranges(lidar_scan)

        # Playing with lidar data
        #rospy.loginfo(lidar_scan[int(len(lidar_scan)/2)])

        if len(groups) != 0:
            self.send_target_angle(min(groups, key = lambda x: abs(x)))
        else:
            self.send_target_angle(-15)

    def send_target_angle(self, angle):
        '''Added to enable ros publishing of angle error'''
        self.direction_pub.publish(Float64(angle))
             
    def find_available_ranges(self, ranges):
        '''Generates list of possible openings and their respective angles'''
        groups = []
        start = -1
        end = -1
        middle = len(ranges)/2
        # Loops through the lidar data and finds openings.
        #  IF there is nothing infront of me and I already have a start point
        #    go ahead a shift my end point down
        #  ELSE IF there is nothing infront of me, but I do not have a start point
        #    set a start points
        #  ELSE IF if I have a start, end, and there is something infront of me
        #    find how wide the opening is. If it is wide enough, store the angle and
        #     the width of the opening into the groups list.
        for index, _range in enumerate(ranges):
            if start >= 0 and _range > self._avoidance_range:
                end = index
            elif _range > self._avoidance_range:
                start = index
            elif start >= 0 and end >= 0:
                theta = np.radians((end - start) * self._angular_resolution)
                cord = np.sqrt(ranges[start]**2 + ranges[end]**2 + 2 * ranges[start] * ranges[end] * np.cos(theta))
                if cord > self._min_clearance:
                    error = ((end - ((end - start) / 2)) - middle) * self._angular_resolution
                    groups.append(error)
                start = -1
                end = -1
        return groups

if __name__=='__main__':
    navigator_node = Navigator()
    rospy.spin()     


