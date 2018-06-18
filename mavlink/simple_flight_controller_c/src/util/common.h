/**
 * @file    common.h
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#ifndef DEMO_COMMON_H
#define DEMO_COMMON_H

#include <stdbool.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#define error(msg) \
  do { perror(msg); exit(EXIT_FAILURE); } while (0)

#define MICRO_PER_SEC 1E6
#define SEC_PER_MICRO 1E-6

/**
 * Converts the provided degrees to radians.
 * @param rad The degrees to convert.
 * @return The provided degrees as radians.
 */
double toRad(double deg);

/**
 * Converts the provided radians to degrees.
 * @param rad The radians to convert.
 * @return The provided radians as degrees.
 */
double toDeg(double rad);
#endif //DEMO_COMMON_H
