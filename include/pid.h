#ifndef PID_H
#define PID_H

#include "types.h"

void pid_init();
void pid_reset_integral();
void pid_compute_setpoints(DroneState *drone); // Calcule Expo et AutoLevel
void pid_compute(DroneState *drone); // Calcule la boucle

#endif