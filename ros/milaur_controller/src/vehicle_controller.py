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
from milaur_controller.msg import Wheel_Velocity, Decoder_Data
from milaur_navigation.msg import Sonar_Data


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
    _wheel_vel_hist_length = 25
    
    _sonar_data_threshold = 125   # Threshold for the sonars

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
        
        self.state = 0

        self.targ_angle_history = deque()
        self.target_angle = 0
        self.sonar_data = 7

        # variables for wheel velocity
        self.left_wheel_history = deque()
        self.right_wheel_history = deque()
        self.forward_vel = 0
        self.desired_vel = 0

        # ROS Subscribers
        self.state_sub = rospy.Subscriber('milaur/state', Int16, self.change_state)
        self.angle_error_sub = rospy.Subscriber('milaur/angle_error', Float64, self.got_target_angle)
        self.sonar_data_sub = rospy.Subscriber('milaur/sonar', Sonar_Data, self.got_sonar_data)
        self.decoder_data_sub = rospy.Subscriber('milaur/decoder', Decoder_Data, self.got_decoder_data)

        # ROS Publishers
        self.state_pub = rospy.Publisher('milaur/state', Int16, queue_size=1)
        self.wheel_velocity_pub = rospy.Publisher('milaur/wheel_velocity', Wheel_Velocity, queue_size=1)

        rospy.sleep(2)  # Sleep while waiting for the publisher to make things happen

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

        # If we're not ready, continually send a motor stop message
        if len(self.targ_angle_history) < self._targ_angle_hist_length//2:
            self.send_wheel_vel(0, 0)
            d = rospy.Duration(2, 0)
            rospy.sleep(d)
            print "Logging target angle"
            return

        if self.state == 1:
            # Don't do anything here. Vision will change to state 2.
            self.send_wheel_vel(0, 0)
            pass

        elif self.state == 2:
            # PID Control
            # Angle error
            angle_error = self.target_angle
            right_wheel_vel = 0
            left_wheel_vel = 0

            if np.abs(angle_error) > np.radians(5):
                # Approximate angular velocity
                angular_velocity = np.average(np.diff(self.targ_angle_history))
                angular_integral = np.trapz(self.targ_angle_history)

                # PID Gains, play with these if the robot jitters
                p_gain = 20 # 0.7
                d_gain = 10 # 0.3
                i_gain = 0.5 # .01
                correction_const = 1.0

                # A positive angle error should induce a positive turn, and the opposite
                tau = (p_gain * angle_error) + (d_gain * angular_velocity) + (i_gain * angular_integral)
                desired_torque = correction_const * tau

                #self.send_wheel_vel(desired_torque, -desired_torque)
                right_wheel_vel = desired_torque
                left_wheel_vel = -desired_torque
                print "Making a turn with a controller effort of ", desired_torque 
            else:
                right_wheel_vel = 0
                left_wheel_vel = 0
                print "Going forward"

            self.send_wheel_vel(right_wheel_vel, left_wheel_vel)

            '''
            # Proportional control for forward velocity
            # This is temp for now, need to get working
            if(self.sonar_data == -1):
                self.send_wheel_vel(0, 0)
                return

            if (self.sonar_data != 0):
                # If there is something in front of me, slow down
                self.desired_vel = int(np.clip(self.desired_vel - 1, 0, len(self.forward_vel) - 1))
            else:
                # If there is not something in front of me, speed up
                self.desired_vel = int(np.clip(self.desired_vel + 1, 0, len(self.forward_vel) - 1))

            print "The desiered vel index: ", self.desired_vel
            print "The desiered vel: ", self.forward_vel[self.desired_vel]

            if (self.forward_vel[self.desired_vel] == 0):
                self.send_wheel_vel(right_wheel_vel, left_wheel_vel)
            else:
                # If we are going forward, scale wheel vels accordingly
                right_wheel_vel = self.forward_vel[self.desired_vel] + right_wheel_vel
                left_wheel_vel = self.forward_vel[self.desired_vel] + left_wheel_vel
                self.send_wheel_vel(right_wheel_vel, left_wheel_vel)
            '''

    def send_wheel_vel(self, right_wheel, left_wheel):
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
        def rectify_wheel_power(value, _min, _max):
            clamped = int(np.clip(value, _min, _max))
            if clamped < 0:
                return float(clamped + 256)
            else:
                return float(clamped)

        left_wheel_vel = rectify_wheel_power(left_wheel, -100, 100)
        right_wheel_vel = rectify_wheel_power(right_wheel, -100, 100)

        msg = Wheel_Velocity(
            right_wheel = right_wheel_vel,
            left_wheel = left_wheel_vel
        )
        self.wheel_velocity_pub.publish(msg)

    def got_target_angle(self, msg):
        '''Target angle should be published at a constant rate
        These angles should be on the interval [-pi, pi]
        The controller will attempt to rotate the robot until the angle error is zero
        '''
        if np.abs(msg.data) > np.radians(6):
            target_angle = msg.data
        else:
            target_angle = 0

        self.target_angle = target_angle
        if len(self.targ_angle_history) > self._targ_angle_hist_length:
            self.targ_angle_history.popleft()
        self.targ_angle_history.append(target_angle)

    def got_sonar_data(self, msg):
        ''' Sonar data is published already converted into cm
        The minimum sonar value should be 20cm
        The Ccontroller will threshold the sonar at 100cm
        '''
        sonar_data = 0
        if(msg.left_sonar < self._sonar_data_threshold):
            sonar_data |= 4
        if(msg.middle_sonar < self._sonar_data_threshold):
            sonar_data |= 2
        if(msg.right_sonar < self._sonar_data_threshold):
            sonar_data |= 1


        if(msg.left_sonar < 75 or msg.middle_sonar < 75 or msg.right_sonar < 75):
            sonar_data = -1

        self.sonar_data = sonar_data

    def got_decoder_data(self, msg):
        msg.right_wheel
        msg.left_wheel

    def change_state(self, msg):
        '''
        Simple callback function to change states
        '''
        state = msg.data
        self.state = state
        print "Changing State: ", state


if __name__=='__main__':
    controller = Controller()
    while(not rospy.is_shutdown()):
        try:
            controller.run()

            # Wait a bit
            rospy.sleep(0.2)
        except rospy.ROSInterruptException:
            exit()
        
