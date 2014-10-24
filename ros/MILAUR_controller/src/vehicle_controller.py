#!/usr/bin/env python
from __future__ import division
## Math
import numpy as np
import math
# Data Structures
from collections import deque
## Ros/tools
import tf
from tf import transformations as tf_trans
import rospy
## Ros msgs
from std_msgs.msg import Header, Int16, Bool, String
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3
from MILAUR_xmega_driver.msg import XMega_Message

def print_in(f):
    '''Shitty decorator for printing function business'''
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name)
        result = f(*args, **kwargs)
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)


def xyzw_array(quaternion):
    '''Convert quaternion to non-shit array'''
    xyzw_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return(xyzw_array)


class Controller(object):
    _targ_angle_hist_length = 25

    def __init__(self):
        '''This is the controller object for the MIL Advanced 
         Utility vehicle

         Purpose:
            Take in a desired vehicle angle and point the robot at that angle

         Function:
            We have no angle feedback, so we will take an angle, and then attempt
             to point the robot at that angle until the desired angle becomes near zero

        '''
        rospy.init_node('vehicle_controller')
        self.targ_angle_history = deque()
        self.target_angle = 0
        self.xmega_pub = rospy.Publisher('MILAUR/send_xmega_msg', XMega_Message, queue_size=1, latch=True)
        self.xmega_pub.publish(
            XMega_Message(
                type=String('robot_start'),
                empty_flag=Bool(True),
            )
        )
        while(1):
            try:
                if len(self.targ_angle_history) < self._targ_angle_hist_length//2:
                    self.xmega_pub.publish(
                       XMega_Message(
                        type=String('motors'),
                        data=String('\x00\x00'),
                        empty_flag=Bool(False),
                        )
                    )
                    continue
            except rospy.ROSInterruptException:
                break
        
            # PID Control
            # Angle error
            angle_error = self.target_angle
            # Approximate angular velocity
            angular_velocity = np.average(np.diff(self.targ_angle_history))
            angular_integral = np.trapz(self.targ_angle_history)

            p_gain = 2.0
            d_gain = 1.0
            i_gain = 0.2
            correction_const = 1.0
            tau = (p_gain * angle_error) + (d_gain * angular_velocity) + (i_gain * angular_integral)

            desired_torque = correction_const * tau

            # Wait a bit
            rospy.sleep(0.08)

    def send_wheel_vel(self, left_wheel, right_wheel):
        '''Send wheel velocities to XMega
        Output ranges:
            [-100,   0] = backwards
            [0,    100] = forwards
        1st byte is left motor, 2nd byte is right motor
        '''
        left_wheel_rectified = np.clip(left_wheel, -100, 100)
        right_wheel_rectified = np.clip(right_wheel, -100, 100)
        left_wheel_char = chr(left_wheel_rectified + 256)
        right_wheel_char = chr(right_wheel_rectified + 256)

        msg = XMega_Message(
            type=String('motors'),
            data=String(left_wheel_char + right_wheel_char),
        )

        self.xmega_pub.publish(msg)

    def got_target_angle(self, msg):
        '''Target angle should be published at a constant rate'''
        target_angle = msg.data
        self.target_angle = target_angle
        if len(self.targ_angle_history) > self._targ_angle_hist_length:
            self.targ_angle_history.popleft()
        self.targ_angle_history.append(target_angle)


if __name__=='__main__':
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass
        
