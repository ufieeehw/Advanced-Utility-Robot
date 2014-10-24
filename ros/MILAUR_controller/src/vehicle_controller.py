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
from std_msgs.msg import Header, Int16, Bool, String, Float64
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
    _targ_angle_hist_length = 25  # Length of deque for derivative and integral PID terms

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
        self.angle_error_sub = rospy.Subscriber('MILAUR/angle_error', Float64, self.got_target_angle)
        self.xmega_pub = rospy.Publisher('MILAUR/send_xmega_msg', XMega_Message, queue_size=10)
        rospy.sleep(2)  # Sleep while waiting for the publisher to make things happen
        self.xmega_pub.publish(
            XMega_Message(
                type=String('robot_start'),
                empty_flag=Bool(True),
            )
        )

    def run(self):
        '''Main loop of the controller
        Determines how to control the vehicle based on the published angle error

        Where +X defines the front of the robot...
                              +X
               Negative Angles | Positive Angles
        (-pi/2)________________|________________ (pi/2)
                               |
                          (-pi or pi)
        '''
        # If we't not ready, continually send a motor stop message
        if len(self.targ_angle_history) < self._targ_angle_hist_length//2:
            self.send_wheel_vel(0, 0)
            d = rospy.Duration(2, 0)
            rospy.sleep(d)
            return
        # PID Control
        # Angle error
        angle_error = self.target_angle
        if angle_error > np.radians(5):
            # Approximate angular velocity
            angular_velocity = np.average(np.diff(self.targ_angle_history))
            angular_integral = np.trapz(self.targ_angle_history)

            # PID Gains, play with these if the robot jitters
            p_gain = 0.7
            d_gain = 0.3
            i_gain = 0.1
            correction_const = 1.0
            # A positive angle error should induce a positive turn, and the opposite
            tau = (p_gain * angle_error) + (d_gain * angular_velocity) + (i_gain * angular_integral)
            desired_torque = correction_const * tau
        else:
            desired_torque = 0
        self.send_wheel_vel(desired_torque, -desired_torque)

    def send_wheel_vel(self, left_wheel, right_wheel):
        '''Send wheel velocities to XMega
        Output ranges:
            [-100,   0] = backwards
            [0,    100] = forwards

            Table:
            | Left Direction | Right Direction | Resultant Motion | Angular Direction |
            |________________|_________________|__________________|___________________|
            | Forward        | Forward         | Forward          | None              |
            | Forward        | Reverse         | Turn Right       | Positive          |
            | Reverse        | Forward         | Turn Left        | Negative          |

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
        '''Target angle should be published at a constant rate
        These angles should be on the interval [-pi, pi]
        The controller will attempt to rotate the robot until the angle error is zero
        '''
        target_angle = msg.data
        self.target_angle = target_angle
        if len(self.targ_angle_history) > self._targ_angle_hist_length:
            self.targ_angle_history.popleft()
        self.targ_angle_history.append(target_angle)


if __name__=='__main__':
    controller = Controller()
    while(not rospy.is_shutdown()):
        try:
            controller.run()
            # Wait a bit
            rospy.sleep(0.2)
        except rospy.ROSInterruptException:
            exit()
        
