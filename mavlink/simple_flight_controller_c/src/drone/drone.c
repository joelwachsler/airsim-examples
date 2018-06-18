/**
 * @file    drone.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include "drone.h"
#include <pthread.h>

pthread_mutex_t drone_state_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t desired_drone_mutex = PTHREAD_MUTEX_INITIALIZER;

drone_state drone = {
  .motors = {
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f,
  },
  .armed = false,
  .ground_alt = 0,
  .home_lat = 0,
  .home_lon = 0,
};

desired_state desired_drone = {
    .pitch_setpoint = 0,
    .roll_setpoint = 0,
    .yaw_setpoint = 0,
    .alt_setpoint = 0,
    .lat = 0,
    .lon = 0,
};

pthread_mutex_t drone_sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
mavlink_hil_sensor_t sensor = {};
mavlink_hil_gps_t gps = {};

void arm() {
  pthread_mutex_lock(&drone_state_mutex);
  drone.armed = true;
  pthread_mutex_unlock(&drone_state_mutex);
}

void disarm() {
  pthread_mutex_lock(&drone_state_mutex);
  drone.armed = false;

  for (int i = 0; i < MOTORS; i++) {
    drone.motors[i] = -1.0f;
  }
  pthread_mutex_unlock(&drone_state_mutex);
}

void set_desired_coordinates(double lat, double lon) {
  pthread_mutex_lock(&desired_drone_mutex);
  desired_drone.lat = lat;
  desired_drone.lon = lon;
  pthread_mutex_unlock(&desired_drone_mutex);
}

void set_home_coordinates(double lat, double lon) {
  pthread_mutex_lock(&drone_state_mutex);
  drone.home_lat = lat;
  drone.home_lon = lon;
  pthread_mutex_unlock(&drone_state_mutex);
}

void update_motor_state(float new_motor_pwm[MOTORS]) {
  pthread_mutex_lock(&drone_state_mutex);
  for (int i = 0; i < MOTORS; i++) {
    drone.motors[i] = new_motor_pwm[i];
  }
  pthread_mutex_unlock(&drone_state_mutex);
}

void set_ground_alt(int32_t alt) {
  pthread_mutex_lock(&drone_state_mutex);
  drone.ground_alt = alt;
  pthread_mutex_unlock(&drone_state_mutex);
}

void set_roll_angle(double setpoint) {
  pthread_mutex_lock(&desired_drone_mutex);
  desired_drone.roll_setpoint = setpoint;
  pthread_mutex_unlock(&desired_drone_mutex);
}

void set_pitch_angle(double setpoint) {
  pthread_mutex_lock(&desired_drone_mutex);
  desired_drone.pitch_setpoint = setpoint;
  pthread_mutex_unlock(&desired_drone_mutex);
}

void set_yaw_angle(double setpoint) {
  pthread_mutex_lock(&desired_drone_mutex);
  desired_drone.yaw_setpoint = setpoint;
  pthread_mutex_unlock(&desired_drone_mutex);
}

void set_altitude(float height) {
  pthread_mutex_lock(&desired_drone_mutex);
  desired_drone.alt_setpoint = height;
  pthread_mutex_unlock(&desired_drone_mutex);
}



