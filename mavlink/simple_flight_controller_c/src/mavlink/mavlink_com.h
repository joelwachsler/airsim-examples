/**
 * @file    mavlink_com.h
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#ifndef DEMO_MAVLINK_COM_H
#define DEMO_MAVLINK_COM_H

#include "mavlink/common/mavlink.h"

/**
 * Sends the provided MAVLink message to the latest node who sent us a message.
 * @param msg The MAVLink message to send.
 */
void send_mavlink_message(const mavlink_message_t* msg);

/**
 * Reads a number of bytes from the communication method.
 * @param buffer The buffer to store bytes in.
 * @param bytes The number of bytes to read from the socket.
 */
void read_bytes(uint8_t* buffer, uint16_t bytes);

/**
 * Initializes the UDP socket.
 * This function has to be called before trying to use any other function defined in this file.
 * @param port The port to listen to.
 * @returns The socket created.
 */
int init_com(uint16_t port);

#endif //DEMO_MAVLINK_COM_H
