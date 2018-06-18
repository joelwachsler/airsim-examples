/**
 * @file    drone.h
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#ifndef SIMPLE_FLIGHT_CONTROLLER_C_DRONE_H
#define SIMPLE_FLIGHT_CONTROLLER_C_DRONE_H

#include <pthread.h>
#include <stdbool.h>
#include <mavlink/common/mavlink.h>

extern pthread_mutex_t drone_sensor_mutex;
extern pthread_mutex_t drone_state_mutex;
extern pthread_mutex_t desired_drone_mutex;

#define MOTORS 4

#define NE_MOTOR 0 // CCW
#define SW_MOTOR 1 // CCW
#define NW_MOTOR 2 // CW
#define SE_MOTOR 3 // CW

struct {
  float motors[MOTORS];
  bool armed;
  double home_lat;
  double home_lon;
  int32_t ground_alt;
} typedef drone_state;

struct {
    double roll_setpoint;
    double pitch_setpoint;
    double yaw_setpoint;
    double alt_setpoint;
    double lat;
    double lon;
} typedef desired_state;

extern drone_state drone;
extern desired_state desired_drone;

// Struct for sensor messages
extern mavlink_hil_sensor_t sensor;
// Struct for GPS messages
extern mavlink_hil_gps_t gps;

/**
 * Arms the drone.
 */
void arm();

/**
 * Disarms the drone.
 */
void disarm();

/**
 * Updates the lat and lon of the drone.
 * @param lat The new latitude.
 * @param lon The new longitude.
 */
void set_desired_coordinates(double lat, double lon);

/**
 * Updates the home coordinates of the drone.
 * @param lat The new latitude home.
 * @param lon The new longitude home.
 */
void set_home_coordinates(double lat, double lon);

/**
 * Updates the PWM of each motor.
 * @param new_motor_pwm The new motor state.
 */
void update_motor_state(float new_motor_pwm[MOTORS]);

/**
 * Sets the ground altitude.
 * @param alt The altitude to set.
 */
void set_ground_alt(int32_t alt);

#endif //SIMPLE_FLIGHT_CONTROLLER_C_DRONE_H

/*
 * Sets the roll setpoint
 * @param  The altitude to set.
 */
void set_roll_angle(double roll);

void set_yaw_angle(double yaw);

void set_pitch_angle(double pitch);

void set_altitude(float height);