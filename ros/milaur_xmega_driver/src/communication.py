#!/usr/bin/python
from __future__ import division # Make all division floating point division

# Serial
import serial
import threading
import numpy

# Math
import numpy as np

# Ros
import rospy
import tf.transformations as tf_trans
import tf
import threading
import time

# Ros Msgs
from std_msgs.msg import Header, Float32, Float64, String
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, Vector3
from milaur_xmega_driver.msg import XMega_Message


lock = threading.Lock()

def thread_lock(function_to_lock):
    '''thread_lock(function) -> locked function 
    Thread locking decorator
        If you use this as a decorator for a function, it will apply a threading lock during the execution of that function,
        Which guarantees that no ROS callbacks can change the state of data while it is executing. This
            is critical to make sure that a new message being sent doesn't cause a weird serial interruption
    '''
    def locked_function(*args, **kwargs):
        # Get threading lock
        lock.acquire()
        # Call the decorated function as normal with its input arguments
        result = function_to_lock(*args, **kwargs)
        # Release the lock
        lock.release()
        # Return, pretending the function hasn't changed at all
        return result
    # Return the function with locking added
    return locked_function


class Communicator(object):
    def __init__(self, port, baud_rate, msg_sub_topic='robot/send_xmega_msg', verbose=True):
        '''Superclass for XMega communication
        Purpose: Communicate with an XMega via serial link
        Function:
            Read (Messages FROM Xmega):
                Loop permanently, listening for a serial message - interpret that message based on
                 predetermined parameters defined in the message type, defined in [1] and described
                 in its readme.
            Write (Messages TO Xmega):
                Silently listen for something published on the ROS message listener topic
                The ROS message is composed of a type and a data field - defined in [2]
                 'type' determines the action (also called message type), and the data field
                 generally is the exact number that will be sent to the xmega. 
                
            The action_dict maps the hex message type to the function that should be called when a message
             of that type is recieved.
        Underdefined:
            - How the length of an outgoing message is determined from a ROS Topic
            - What is a 'poll_message' and what is a 'data_message' and if there needs to be a difference
        Notes:
            This was designed to work with the communication protocol as we defined it. It does not do anything that relies
            on the specific XMega-side implementation of the protocol.
            At some point, we should consider building a simpler XMega implementation of the protocol, which relies less on malloc
            Proposal: Each item defines its own message length independent of the defined type. OR: The type field fully encodes 
                length (0-255 bytes)
                -- Would this improve things?
        Bibliography:
            [1] https://github.com/ufieeehw/IEEE2015/tree/master/xmega
            [2] https://github.com/ufieeehw/IEEE2015/tree/master/ros/ieee2015_xmega_driver/msg
        '''
        self.verbose = verbose

        # ROS Setup
        rospy.init_node('XMega_Connector')
        # Messages being sent from ROS to the XMega
        self.send_msg_sub = rospy.Subscriber(msg_sub_topic, XMega_Message, self.got_ros_msg)
        try:
            self.serial = serial.Serial(port, baud_rate)
        except(serial.serialutil.SerialException):
            print("ieee2015_communicator could not open port " + port + 
                "\nIf you have the Xmega plugged in, try setting up the Udev rules"
            )
        # Defines which action function to call on the received data
        self.action_dict = {
        }
        # Defines the relationship between poll_message names and the hex name
        self.poll_messages = {
            'example_poll_msg': '0F'
        }
        # First two bits determine the length of the message
        self.byte_type_defs = {
            0b00000000: 0,
            0b01000000: 1,
            0b10000000: 2,
            0b11000000: None,
        }

    def err_log(self, *args):
        '''Print the inputs as a list if class verbosity is True
        '''
        if self.verbose:
            print args

    @thread_lock
    def got_ros_msg(self, msg):
        '''Only supports 2 byte and empty messages right now!'''
        self.err_log("Got poll message of type ", msg.type)
        if msg.empty_flag:
            self.write_packet(msg.type)
        else:
            self.write_packet(msg.type, msg.data)

    @thread_lock
    def send_keep_alive(self):
        '''Send a keep-alive message
        Critically, this includes a threading lock'''
        self.write_packet('keep_alive')

    def read_packets(self):
        '''read_packets
        Function:
            Permanently loops, waiting for serial messages from the XMega, and then calls 
             the appropriate action function
        Notes:
            This does not handle desynchronization with the microcontroller
        '''
        type_length = 1  # Bytes
        length_length = 1  # Bytes
        type_mask =  0b11000000
        error_mask = 0b00110000

        # Initialize time
        old_time = time.time()

        while True:
            # Timed watchdog messages
            new_time = time.time()
            if (new_time - old_time) > 0.5:
                old_time = new_time
                self.send_keep_alive()

            # Handle the first byte, determining type
            unprocessed_type = self.serial.read(type_length)
            self.err_log("shitty type ", unprocessed_type)
            msg_type = ord(unprocessed_type)
            msg_byte_type = msg_type & type_mask
            b_error = (msg_type & error_mask) == error_mask
            self.err_log('Recieving message of type', msg_type)

            # Message of known length
            if msg_byte_type in self.byte_type_defs.keys():
                msg_length = self.byte_type_defs[msg_byte_type]
                if msg_length == 0:
                    msg_data = None
                elif msg_length > 0:
                    msg_data = self.serial.read(msg_length)
                    
                if msg_type in self.action_dict.keys():
                    action_function = self.action_dict[msg_type]
                    action_function(msg_data)
                else:
                    self.err_log("No action fun for ", msg_type)

            # N-Byte Message
            elif msg_type in self.action_dict.keys():
                action_function = self.action_dict[msg_type]
                self.err_log('Recognized type as', action_function.__name__)

                msg_length = self.serial.read(length_length)
                msg_data = self.serial.read(msg_length)
                self.err_log("Message content:", msg_data)

                action_function(msg_data)

            # Failure
            else:
                self.err_log('Did not recognize type', msg_type)

        if msg_byte_type in self.byte_type_defs.keys():
            msg_length = self.byte_type_defs[msg_byte_type]
            if msg_length > 0:
                msg_data = None
            else:
                msg_data = self.serial.read(msg_length)
            action_function = self.action_dict[msg_type]
            action_function(msg_data)

        # N-Byte Message
        elif msg_type in self.action_dict.keys():
            action_function = self.action_dict[msg_type]

            self.err_log('Recognized type as', action_function.__name__)
            msg_length = self.serial.read(length_length)
            msg_data = self.serial.read(msg_length)
            self.err_log("Message content:", msg_data)
            action_function(msg_data)

    def write_packet(self, _type, data=None):
        '''write_packet(self, _type, data=None)
        Function:
            This [effectively] listens to ROS messages on either the
        Notes:
            type is _type because "type" is a python protected name
        '''
        self.err_log("Processing message of type ", _type)
        if _type in self.poll_messages.keys():
            self.err_log("Write type recognized as a polling message")
            write_data = self.poll_messages[_type]
            self.err_log("Writing as ", write_data)
            self.serial.write(chr(write_data))
            if data is not None:
                self.err_log("Data, ")
                for character in data:
                    self.err_log("writing character ", character)
                    self.serial.write(character)
            else:
                self.err_log("No other data to write")
        else:
            self.err_log("Write type not recognized")


class milaur_Communicator(Communicator):
    def __init__(self, port='/dev/ttyUSB0', baud_rate=256000):
        '''milaur sub-class of the broader UFIEEE XMega Communicator class
        XMega Sensor Manifest:
            - 3x: Sonars
        XMega Actuator Manifest:
            - 2x: Wheel Motors (Set Motor Powers)
        '''
        super(self.__class__, self).__init__(port, baud_rate, msg_sub_topic='milaur/send_xmega_msg')

        # Define your publisher here
        #self.accel_data_pub = rospy.Publisher('milaur/nunchuck', Imu, queue_size=1)

        # For messages of known length, get the length from this table
        # Determine which action function to call on the received data
        self.action_dict.update({
            # C0 + $num_bytes + $message
            0xF0: self.got_xmega_error,
            0xEF: self.towbot_sonar_echo,
            #0xC0: self.towbot_nunchuck_echo,
            0x40: self.got_test,
        })
        self.poll_messages.update({
            # 'poll_imu': 0x01,
            'robot_start': 0x02,
            'motors': 0x80,
            'debug': 0x40,
        })

    def got_xmega_error(self, msg_data):
        self.err_log("Got error,", msg_data)

    def got_test(self, msg_data):
        print "Recieved test!"
        if msg_data is not None:
            print "Data:", msg_data


#    def towbot_nunchuck_echo(self, msg_data):
#        '''Towbot nunchuck demo
#        TODO: Rosify this
#            byte 1: Stick X (80 is middle)
#            byte 2: Stick Y (80 is middle)
#            byte 3: Acc X
#            byte 4: Acc Y
#            byte 5: Acc Z
#            byte 6: last 2 bits are C and Z buttons
#        '''
#        msg_format = ['Stick X', 'Stick Y', 'Acc X', 'Acc Y', 'Acc Z', 'Bullshit']
#        for character, meaning in zip(msg_data, msg_format):
#            print meaning + ':', character
#        self.err_log("We're up in this shit towbot nunchuck!")

#        self.write_packet('init_towbot_poll')

    def towbot_sonar_echo(self, msg_data):
        '''TowBot sonar data
        TODO: Rosify this
            byte 1: Left Sonar High
            byte 2: Left Sonar Low
            byte 3: Middle Sonar High
            byte 4: Middle Sonar Low
            byte 5: Right Sonar High
            byte 6: Right Sonar Low
        '''
        msg_format = ['Stick X', 'Stick Y', 'Acc X', 'Acc Y', 'Acc Z', 'Bullshit']
        for character, meaning in zip(msg_data, msg_format):
            print meaning + ':', character
        self.err_log("We're up in this shit towbot nunchuck!")

        #self.write_packet('init_towbot_poll')


if __name__=='__main__':
    Comms = milaur_Communicator(port='/dev/ttyUSB0')
    Comms.read_packets()
    rospy.spin()
