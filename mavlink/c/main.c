#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <stdbool.h>

#include "mavlink/common/mavlink.h"

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
int sockfd;
ssize_t n;

struct sockaddr_in server;
socklen_t socklen = sizeof(struct sockaddr_in);
const int port = 14560;

void error(char *msg) {
  perror(msg);
  exit(0);
}

/**
 * Wait for a GPS message to be received from the server.
 * @return a decoded GPS struct message.
 */
mavlink_hil_gps_t get_gps_msg() {
  while (1) {
    n = recvfrom(sockfd, buf, 1024, 0, (struct sockaddr *)&server, &socklen);
    if (n < 0) {
      error("Failed to recvfrom");
    }

    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < n; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_HIL_GPS) {
          mavlink_hil_gps_t gps;
          mavlink_msg_hil_gps_decode(&msg, &gps);
          return gps;
        }
      }
    }
  }
}

/*
 * Updates the pwm of the four motor controls.
 */
void update_motor_pwm(float pwm[]) {
  mavlink_message_t msg;
  memset(buf, 0, MAVLINK_MAX_PACKET_LEN);

  const float controls[] = {
    pwm[0],
    pwm[1],
    pwm[2],
    pwm[3],
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f
  };

  size_t msglen = mavlink_msg_hil_actuator_controls_pack(0, 200, &msg, 0, controls, 0, 0);
  mavlink_msg_to_send_buffer(buf, &msg);
  n = sendto(sockfd, buf, msglen, 0, (struct sockaddr *)&server, socklen);

  if (n < 0) {
    error("Failed to send message");
  }
}

/*
 * Initializes a socket on the specified port.
 */
void socket_init() {
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    error("Opening socket");
  }

  memset(&server, 0, sizeof(struct sockaddr_in));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(port);

  if (bind(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0) {
    error("Binding error");
  }
}

int main(int argc, char *argv[]) {
  // Initialize the socket
  socket_init();

  // Initialize our ground altitude and set a target altitude
  int ground = get_gps_msg().alt;
  int target_alt = 2500;

  //Create an array to store our values for the pwm of the motors.
  float pwm[4];

  while (1) {
    //Retrieve a gps message to get our current altitude
    mavlink_hil_gps_t gps = get_gps_msg();
    int height = gps.alt;
    int diff = height - ground;
    printf("Current height: %fm\n", (float)diff / 1000.0);

    if (diff > target_alt)
    {
      pwm[0] = 0.48;
      pwm[1] = 0.48;
      pwm[2] = 0.48;
      pwm[3] = 0.48;
      printf("Updating the pwm to %f\n", 0.48);
    }
    else
    {
      pwm[0] = 0.50;
      pwm[1] = 0.50;
      pwm[2] = 0.50;
      pwm[3] = 0.50;
      printf("Updating the pwm to %f\n", 0.50);
    }
    update_motor_pwm(pwm);
  }
}
