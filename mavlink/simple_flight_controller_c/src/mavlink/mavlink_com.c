/**
 * @file    mavlink_com.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "mavlink_com.h"
#include "mavlink_msg_handler.h"
#include "../util/common.h"

// Keep track of the node to send to.
struct sockaddr_in server = {};
int sockfd;

int init_com(uint16_t port) {
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    error("Opening socket");
  }

  // zero the server struct
  memset(&server, 0, sizeof(struct sockaddr_in));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(port);

  if (bind(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0) {
    error("Binding error");
  }

  printf("Communication initialized!\n");

  return sockfd;
}

void send_mavlink_message(const mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t buflen = mavlink_msg_to_send_buffer(buf, msg);

  socklen_t socklen = sizeof(struct sockaddr);
  ssize_t bytes_sent = sendto(sockfd, buf, buflen, 0, (struct sockaddr *) &server, socklen);

  if (bytes_sent < 0) {
    error("Failed to send MAVLink message over the socket...");
  }
}

void read_bytes(uint8_t *buffer, const uint16_t bytes) {
  socklen_t len = sizeof(struct sockaddr_in);

  ssize_t bytes_received = recvfrom(sockfd, buffer, bytes, 0, (struct sockaddr *) &server, &len);

  if (bytes_received < 0) {
    error("Failed to recvfrom");
  }
}
