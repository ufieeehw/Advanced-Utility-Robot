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
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3


class Controller(object):
	def __init__(self):
		'''This is the controller object for the MIL Advanced 
 		 Utility vehicle
 		'''
		pass