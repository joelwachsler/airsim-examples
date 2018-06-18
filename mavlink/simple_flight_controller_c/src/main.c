/**
 * @file    main.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda
 * @date    2018-05-13
 */

#include <pthread.h>
#include <unistd.h>

#include "mavlink/mavlink_com.h"
#include "mavlink/mavlink_msg_handler.h"
#include "pid/calculate_pids.h"
#include "cmd_handler/cmd_handler.h"
#include "util/common.h"
#include "drone/drone.h"

//void logger() {
//  FILE *f = NULL;
//
//  while (1) {
//    if (f == NULL && drone.armed) {
//       f = fopen("logging.txt", "a");
//    }
//
//    while (drone.ground_alt == 0) {
//      usleep(1000);
//    }
//
//    if (drone.armed) {
//      if (f == NULL) {
//        printf("Error opening file!\n");
//        exit(1);
//      }
//
//      /* print some text */
//      fprintf(f, "%ld;%f;%f;%f\n", sensor.time_usec, angle_pitch, angle_roll, angle_yaw);
//    }
//
//    if (f != NULL && !drone.armed) {
//      fclose(f);
//    }
//
//    usleep(1000);
//  }
//}

int main(int argc, char *argv[]) {
  // Initialize the communication
  uint16_t port = 14560;

  if (argc == 2) {
    port = (uint16_t) atoi(argv[1]);
  }

  int sock = init_com(port);

  pthread_t threads[4];

//  if (pthread_create(&threads[4], NULL, logger, NULL)) {
//    error("Creating logger");
//  }

  if (pthread_create(&threads[0], NULL, msg_listener, NULL)) {
    error("Creating message listener");
  }

  if (pthread_create(&threads[1], NULL, send_heartbeat_messages, (void *) 1)) {
    error("Creating heartbeat message sender");
  }

  if (pthread_create(&threads[2], NULL, send_motor_state, NULL)) {
    error("Creating motor state sender");
  }

  if (pthread_create(&threads[3], NULL, calculate_pids, NULL)) {
    error("Creating pid controller");
  }

  start_cmd_handler();

  close(sock);
}
