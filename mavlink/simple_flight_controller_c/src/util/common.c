/**
 * @file    common.c
 * @author  Joel Wachsler
 * @author  Daniel Aros Banda 
 * @date    2018-05-13
 */

#include "common.h"

double toRad(double deg) {
  return deg * 0.0174533;
}

double toDeg(double rad) {
  return rad * 57.2958;
}
