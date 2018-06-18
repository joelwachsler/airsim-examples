/**
 * @file    pid.h
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#ifndef DEMO_PID_H
#define DEMO_PID_H

/**
 * Struct holding various information used by the PID controller.
 */
struct {
  const double p_gain;
  const double i_gain;
  const double d_gain;
  const double max;
  double setpoint;
  double i_mem;
  double last_d_error;
} typedef pid;

/**
 * Calculates the PID controller output from the input variables.
 * @param input The input value of the PID.
 * @param curr_pid The PID struct.
 * @return The result of the PID calculation.
 */
double calculate_pid(double input, pid* curr_pid);

#endif //DEMO_PID_H
