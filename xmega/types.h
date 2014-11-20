//add a mesage to the appropriate section
//register at the start of the range for incoming/mixed message types
//register at the end of the range for outgoing only types
/*Message Types, Mask Definitions, and Macros */
#ifndef _IEEE_TYPES_H_
#define _IEEE_TYPES_H_

/* Masks and Bit Definitons 
The comments afterward are parsed by ROS (ieee2015_xmega_driver/src/parse_types)
*/
//Message Data length is first two bits of type field
#define DATA_MASK             0xC0 // mask: data
#define NO_DATA_TYPE          0x00 // msg_length: 0
#define DATA_1B_TYPE          0x40 // msg_length: 1
#define DATA_2B_TYPE          0x80 // msg_length: 2
#define DATA_NB_TYPE          0xC0 // msg_length: None

//Error Types must have 11 as bits 6 and 5
#define ERROR_MASK            0x30 // mask: error
#define ERROR_TYPE            0x30 
#define IS_ERROR_TYPE(x)      ((x & ERROR_MASK) == ERROR_TYPE)


/* Message Type Definitions */
//NO_DATA_TYPE messages [0x01-0x2F]
#define KILL_TYPE             0x01 // out: kill
#define START_TYPE            0x02 // out: start
#define KEEP_ALIVE_TYPE       0x03 // out: keep_alive
#define IMU_NOTIFY_TYPE       0x04 // out: poll_imu


//DATA_1B_TYPE messages [0x40-0x6F]
#define DEBUG_TYPE            0x40 // in: xmega_debug; out: ros_debug


//DATA_2B_TYPE messages [0x80-0xAF]
#define TOWBOT_TYPE           0x80


//DATA_NB_TYPE messages [0xC0-0xEF]

#define SONAR_DATA_TYPE         0xEF // in: sonar_data


/*Error Type Definitions (should represent an entire subsystem) */
//NO_DATA_TYPE errors [0x30-0x3F] (and also OK)
#define OK                    0x00 // in: xmega_ok
#define MESSAGE_ERROR_TYPE    0x30 // in: message_error
#define BUFFER_ERROR_TYPE     0x31 // in: buffer_error


//DATA_1B_TYPE errors [0x70-0x7F]


//DATA_2B_TYPE errors [0xB0-0xBF]


//DATA_NB_TYPE errors [0xF0-0xFF]
#define VECTOR_ERROR_TYPE     0xF0 // in: vector_error


/* TODO: FILL IN TYPES */

#endif

