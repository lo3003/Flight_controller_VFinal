#ifndef MOTORS_H
#define MOTORS_H

#include "types.h"

void motors_init();
void motors_stop(); // Force 1000
void motors_mix(DroneState *drone);
void motors_write(); // Génère le PWM

#endif