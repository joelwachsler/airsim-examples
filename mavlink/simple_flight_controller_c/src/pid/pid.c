/**
 * @file    pid.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "pid.h"
#include "../util/common.h"
#include "../mavlink/mavlink_com.h"
#include "../pid/calculate_pids.h"

double calculate_pid(double input, pid* curr_pid) {
  double pid_error_temp = input - curr_pid->setpoint;

  curr_pid->i_mem += curr_pid->i_gain * pid_error_temp;

  if(curr_pid->i_mem > curr_pid->max) curr_pid->i_mem = curr_pid->max;
  else if(curr_pid->i_mem < curr_pid->max * -1) curr_pid->i_mem = curr_pid->max * -1;

  double pid_output = curr_pid->p_gain * pid_error_temp + curr_pid->i_mem + curr_pid->d_gain * (pid_error_temp - curr_pid->last_d_error);

  if(pid_output > curr_pid->max) pid_output = curr_pid->max;
  else if(pid_output < curr_pid->max * -1) pid_output = curr_pid->max * -1;

  curr_pid->last_d_error = pid_error_temp;

  return pid_output;
}
