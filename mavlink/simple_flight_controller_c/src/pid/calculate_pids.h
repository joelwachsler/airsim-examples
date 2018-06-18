/**
 * @file    calculate_pids.h
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#ifndef DEMO_CALCULATE_PIDS_H
#define DEMO_CALCULATE_PIDS_H

#include "pid.h"

#define PID_CALC_HZ 10

extern pid pitch;
extern pid roll;
extern pid yaw;

extern float angle_roll;
extern float angle_pitch;
extern float angle_yaw;

extern pid altitude;

extern pid lat_pid;
extern pid lon_pid;

void* calculate_pids(void* args);

#endif //DEMO_CALCULATE_PIDS_H
