/**
 * @file    mavlink_msg_handler.h
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#ifndef SIMPLE_CONTROLLER_C_MAVLINK_MSG_HANDLER_H
#define SIMPLE_CONTROLLER_C_MAVLINK_MSG_HANDLER_H

#include <mavlink/common/mavlink.h>
#include <pthread.h>

// The HZ to send the motor values
#define MOTOR_SEND_HZ 100

/**
 * Sends the motor state at the provided rate to AirSim.
 * @param args The rate of which to send the motor info to AirSim.
 */
void* send_motor_state(void* args);

/**
 * Handles MAVLink messages by placing their content in global structs.
 * @param msg The message to place in a global struct.
 */
void handle_mavlink_msg(const mavlink_message_t* msg);

/**
 * Listens for MAVLink messages and updates their corresponding structs.
 * @param args The socket to listen to messages from.
 */
void* msg_listener(void* args);

/**
 * Sends heartbeat messages at a rate according to the provided argument.
 * @param hz The hz to send heartbeat messages (1 = 1 time a second).
 */
void* send_heartbeat_messages(void* hz);

#endif //SIMPLE_CONTROLLER_C_MAVLINK_MSG_HANDLER_H
