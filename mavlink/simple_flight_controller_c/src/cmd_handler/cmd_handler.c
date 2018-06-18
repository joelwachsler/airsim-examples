/**
 * @file    cmd_handler.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include "cmd_handler.h"
#include "../pid/calculate_pids.h"
#include "../mavlink/mavlink_msg_handler.h"
#include "../drone/drone.h"
#include "../util/common.h"
#include "../pid/pid.h"

bool cmd_cmp(const char* input, const char* cmd) {
  return !strncasecmp(input, cmd, strlen(cmd));
}

void cmd_arm() {
  printf("Arming the drone!\n");
  arm();
}

void cmd_disarm() {
  printf("Disarming the drone!\n");
  disarm();
}

void cmd_go_home() {
  set_desired_coordinates(drone.home_lat, drone.home_lon);
  printf("Going to the original latitude and longitude! (lat: %f, lon: %f)\n", drone.home_lat, drone.home_lon);
}

void cmd_land() {
  printf("Landing the drone! (going to the starting altitude)\n");
  set_altitude(0.0);
}

void cmd_gps() {
  printf("lat: %lf, lon: %lf\n", (double) gps.lat / 1E7, (double) gps.lon / 1E7);
}

void cmd_altitude() {
  printf("The current altitude is: %.2fm\n", (gps.alt - drone.ground_alt) / 1000.0);
}

void cmd_flyto(char* input, char* tmp) {
  double lat = 0, lon = 0;

  sscanf(input, "%s %lf %lf", tmp, &lat, &lon);
  //set_desired_coordinates(lat * -1.0 / 1E7, lon / 1E7);
  set_desired_coordinates(lat, lon);

  printf("Flying to lat: %f, lon: %f\n", lat, lon);
}

void cmd_takeoff(char* input, char* tmp) {
  float height = 0;
  sscanf(input, "%s %f", tmp, &height);

  printf("Taking off to: %.2fm\n", height);
  set_altitude(height * 1000);
}

void cmd_roll(char* input, char* tmp) {
  float angle = 0;
  sscanf(input, "%s %f", tmp, &angle);

  printf("Rolling %0.2f degrees\n", angle);
  set_roll_angle(toRad(angle));
}

void cmd_pitch(char* input, char* tmp) {
  float angle = 0;
  sscanf(input, "%s %f", tmp, &angle);

  printf("Pitching %0.2f degrees\n", angle);
  set_pitch_angle(toRad(angle));
}

void cmd_yaw(char* input, char* tmp) {
  float angle = 0;
  sscanf(input, "%s %f", tmp, &angle);

  printf("Yawing %0.2f degrees\n", angle);
  set_yaw_angle(toRad(angle));
}

bool running = true;

void cmd_exit() {
  printf("Thank you for flying!\n");
  running = false;

  usleep(MICRO_PER_SEC);
}

void cmd_invalid() {
  printf("Invalid argument!\n");
  printf("The available commands are as follows:\n");

  printf("arm\n");
  printf("disarm\n");
  printf("gohome\n");
  printf("land\n");
  printf("gps\n");
  printf("altitude | alt\n");
  printf("flyto [latitude] [longitude]\n");
  printf("takeoff [altitude in meters]\n");
  printf("pitch [degrees]\n");
  printf("roll [degrees]\n");
  // printf("yaw [degrees]\n");
  printf("exit | q\n");
}

void start_cmd_handler() {
  printf("--- COMMAND HANDLER STARTED ---\n");
  char input[32], tmp[32];

  while (running) {
    printf("> ");
    fflush(stdout);
    fgets(input, 32, stdin);

    if      (cmd_cmp(input, "arm"))      cmd_arm();
    else if (cmd_cmp(input, "disarm"))   cmd_disarm();
    else if (cmd_cmp(input, "gohome"))   cmd_go_home();
    else if (cmd_cmp(input, "land"))     cmd_land();
    else if (cmd_cmp(input, "gps"))      cmd_gps();
    else if (cmd_cmp(input, "altitude")) cmd_altitude();
    else if (cmd_cmp(input, "alt"))      cmd_altitude();
    else if (cmd_cmp(input, "flyto"))    cmd_flyto(input, tmp);
    else if (cmd_cmp(input, "takeoff"))  cmd_takeoff(input, tmp);
    else if (cmd_cmp(input, "pitch"))    cmd_pitch(input, tmp);
    else if (cmd_cmp(input, "roll"))     cmd_roll(input, tmp);
    else if (cmd_cmp(input, "yaw"))      cmd_yaw(input, tmp);
    else if (cmd_cmp(input, "exit"))     cmd_exit();
    else if (cmd_cmp(input, "q"))        cmd_exit();
    else                                 cmd_invalid();
  }
}
