/**
 * @file    calculate_pids.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include <stdio.h>
#include <unistd.h>

#include "calculate_pids.h"
#include "../mavlink/mavlink_msg_handler.h"
#include "pid.h"
#include "../drone/drone.h"
#include "../util/common.h"

#define PID_P_GAIN 0.1125
#define PID_I_GAIN 0.001
#define PID_D_GAIN 0.8
#define PITCH_ROLL_MAX 0.2

pid pitch = {
  .p_gain = PID_P_GAIN,
  .i_gain = PID_I_GAIN,
  .d_gain = PID_D_GAIN,
  .max = PITCH_ROLL_MAX,
  .setpoint = 0.0,
  .i_mem = 0.0,
  .last_d_error = 0.0,
};

pid roll = {
  .p_gain = PID_P_GAIN,
  .i_gain = PID_I_GAIN,
  .d_gain = PID_D_GAIN,
  .max = PITCH_ROLL_MAX,
  .setpoint = 0.0,
  .i_mem = 0.0,
  .last_d_error = 0.0,
};

pid yaw = {
  .p_gain = 5,
  .i_gain = 0.1,
  .d_gain = 35,
  .max = 0.2,
  .setpoint = 0.0,
  .i_mem = 0.0,
  .last_d_error = 0.0,
};

#define ALTITUDE_SCALE 10000

pid altitude = {
  .p_gain = 2,
  .i_gain = 0.0005525,
  .d_gain = 25,
  .max = (0.2 * ALTITUDE_SCALE),
  .setpoint = 0.0,
  .i_mem = 0.0,
  .last_d_error = 0.0,
};

#define LAT_LON_P 500
#define LAT_LON_I 0
#define LAT_LON_D 25000
#define LAT_LON_MAX 0.15

pid lat_pid = {
  .p_gain = LAT_LON_P,
  .i_gain = LAT_LON_I,
  .d_gain = LAT_LON_D,
  .max = LAT_LON_MAX,
  .setpoint = 0.0,
  .i_mem = 0.0,
  .last_d_error = 0.0,
};

pid lon_pid = {
  .p_gain = LAT_LON_P,
  .i_gain = LAT_LON_I,
  .d_gain = LAT_LON_D,
  .max = LAT_LON_MAX,
  .setpoint = 0.0,
  .i_mem = 0.0,
  .last_d_error = 0.0,
};

void wait_for_sensor_values() {
  while (sensor.xacc == 0 && sensor.yacc == 0 && sensor.zacc == 0 || gps.alt == 0) {
    printf("Waiting for sensor data...\n");
    usleep((useconds_t) (MICRO_PER_SEC / 10));
  }
}

float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

struct {
  float x;
  float y;
  float z;
} typedef xyz;

void calc_angle_pitch_roll_yaw(const xyz* gyro, const xyz* acc, const double dt) {
  angle_pitch += gyro->y * dt;
  angle_roll += gyro->x * dt;
  angle_yaw += gyro->z * dt;

  angle_roll -= angle_pitch * sin(gyro->z * dt);
  angle_pitch += angle_roll * sin(gyro->z * dt);

  float acc_total_vector = sqrtf((acc->x * acc->x) + (acc->y * acc->y) + (acc->z * acc->z));
  float angle_pitch_acc = asinf(acc->x / acc_total_vector);
  float angle_roll_acc = asinf(acc->y / acc_total_vector);
  // TODO: This may not be correct
  const float angle_yaw_acc = asinf(acc->z / acc_total_vector);

  // Using a complementary filter to reduce drag
  angle_pitch = (float) (angle_pitch * 0.9996 + angle_pitch_acc * 0.0004);
  angle_roll = (float) (angle_roll * 0.9996 + angle_roll_acc * 0.0004);
  angle_yaw = (float) (angle_yaw * 0.9996 + angle_yaw_acc * 0.0004);
}

float getDronePointingDeg(float xmag, float ymag) {
  return (float) ((atan2f(ymag, xmag) * 180) / M_PI);
}

void* calculate_pids(void* args) {
  const double dt_sleep = MICRO_PER_SEC / PID_CALC_HZ;
  const double dt = dt_sleep * SEC_PER_MICRO;

  wait_for_sensor_values();

  set_ground_alt(gps.alt);

  set_home_coordinates((double) gps.lat / 1E7, (double) gps.lon / 1E7);
  set_desired_coordinates((double) gps.lat / 1E7, (double) gps.lon / 1E7);

  xyz gyro_cache = {};
  xyz acc_cache = {};
  xyz mag_cache = {};

  while (true) {
    if (!drone.armed) {
      usleep(dt_sleep);
      continue;
    }
    // Cache sensor values such that they don't change while the PID calculations are running
    pthread_mutex_lock(&drone_sensor_mutex);
    gyro_cache.x = sensor.xgyro;
    gyro_cache.y = sensor.ygyro;
    gyro_cache.z = sensor.zgyro;

    acc_cache.x = sensor.xacc;
    acc_cache.y = sensor.yacc;
    acc_cache.z = sensor.zacc;

    mag_cache.x = sensor.xmag;
    mag_cache.y = sensor.ymag;
    mag_cache.z = sensor.zmag;

    const int32_t alt = gps.alt - drone.ground_alt;
    const double lat = (double) gps.lat / 1E7;
    const double lon = (double) gps.lon / 1E7;
    pthread_mutex_unlock(&drone_sensor_mutex);

    pthread_mutex_lock(&desired_drone_mutex);
    // Always point to north
    yaw.setpoint = getDronePointingDeg(mag_cache.x, mag_cache.y);

//    pitch.setpoint = desired_drone.pitch_setpoint;
//    roll.setpoint = desired_drone.roll_setpoint;
    altitude.setpoint = desired_drone.alt_setpoint;

    lat_pid.setpoint = desired_drone.lat;
    lon_pid.setpoint = desired_drone.lon;
    pthread_mutex_unlock(&desired_drone_mutex);

    calc_angle_pitch_roll_yaw(&gyro_cache, &acc_cache, dt);

    pitch.setpoint = calculate_pid(lat, &lat_pid);
    roll.setpoint = calculate_pid(lon, &lon_pid) * -1;

    double pid_output_pitch = calculate_pid(angle_pitch, &pitch);
    double pid_output_roll = calculate_pid(angle_roll, &roll);
    double pid_output_yaw = calculate_pid(angle_yaw, &yaw);
    double pid_output_altitude_throttle = calculate_pid(alt, &altitude);

    // Throttle normalization
    pid_output_altitude_throttle *= -1;
    pid_output_altitude_throttle /= ALTITUDE_SCALE;
    pid_output_altitude_throttle += 0.48;

    // Update the motors
    float new_motor_state[MOTORS];
    new_motor_state[NE_MOTOR] = (float) (pid_output_altitude_throttle - pid_output_pitch + pid_output_roll - pid_output_yaw); // CCW
    new_motor_state[SE_MOTOR] = (float) (pid_output_altitude_throttle + pid_output_pitch + pid_output_roll + pid_output_yaw); // CW
    new_motor_state[SW_MOTOR] = (float) (pid_output_altitude_throttle + pid_output_pitch - pid_output_roll - pid_output_yaw); // CCW
    new_motor_state[NW_MOTOR] = (float) (pid_output_altitude_throttle - pid_output_pitch - pid_output_roll + pid_output_yaw); // CW

    update_motor_state(new_motor_state);

    usleep(dt_sleep);
  }
}
