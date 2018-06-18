/**
 * @file    mavlink_msg_handler.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <mavlink/common/mavlink.h>

#include "mavlink_msg_handler.h"
#include "mavlink_com.h"
#include "../util/common.h"
#include "../drone/drone.h"

#define BUF_SIZE 1024
#define SYSTEM_ID 0
#define COMPONENT_ID 200

void* send_motor_state(void *args) {
  printf("Motor state sender started\n");

  mavlink_message_t msg;

  float motors[16];

  for (int i = 0; i < 16; i++) {
    motors[i] = -1.0f;
  }

  ssize_t hz = (ssize_t) (MICRO_PER_SEC / MOTOR_SEND_HZ);

  while (true) {
    pthread_mutex_lock(&drone_state_mutex);
    for (int i = 0; i < MOTORS; i++) {
      motors[i] = drone.motors[i];
    }
    pthread_mutex_unlock(&drone_state_mutex);

    mavlink_msg_hil_actuator_controls_pack(SYSTEM_ID, COMPONENT_ID, &msg, 0, motors, 0, 0);

    send_mavlink_message(&msg);

    usleep(hz);
  }
}

void handle_mavlink_msg(const mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HIL_GPS:
      pthread_mutex_lock(&drone_state_mutex);
      mavlink_msg_hil_gps_decode(msg, &gps);
      pthread_mutex_unlock(&drone_state_mutex);
      break;
    case MAVLINK_MSG_ID_HIL_SENSOR:
      pthread_mutex_lock(&drone_state_mutex);
      mavlink_msg_hil_sensor_decode(msg, &sensor);
      pthread_mutex_unlock(&drone_state_mutex);
      break;
    default:
      break;
  }
}

void* msg_listener(void *args) {
  printf("Message listener started\n");

  uint8_t buf[BUF_SIZE];
  memset(buf, 0, BUF_SIZE);

  mavlink_message_t msg;
  mavlink_status_t status;

  while (true) {
    read_bytes(buf, BUF_SIZE);

    // TODO: Decrease the maximum value of i (a message cannot be shorter than a specified value)
    for (int i = 0; i < BUF_SIZE; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        handle_mavlink_msg(&msg);
      }
    }
  }
}

void* send_heartbeat_messages(void* args) {
  printf("Heartbeat sender initialized!\n");

  ssize_t hz = (ssize_t) args;
  useconds_t hz_sleep = (useconds_t) (MICRO_PER_SEC / hz);

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);

  while (true) {
    send_mavlink_message(&msg);

    usleep(hz_sleep);
  }
}
