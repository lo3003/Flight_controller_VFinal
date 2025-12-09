#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"

DroneState drone;
unsigned long loop_timer;

void setup() {
    Wire.begin();
    TWBR = 12; 
    
    // Init Modules
    radio_init();
    imu_init();
    pid_init();
    motors_init();
    
    // Init State
    drone.current_mode = MODE_SAFE;
    
    // Attente Récepteur actif
    while(drone.channel_3 < 990 || drone.channel_3 > 1020) {
        radio_update(&drone);
        delay(10);
    }
    
    loop_timer = micros();
}

void loop() {
    // 1. Inputs
    radio_update(&drone);
    imu_read(&drone);

    // 2. Machine d'État (FSM)
    switch(drone.current_mode) {
        case MODE_SAFE:
            motors_stop();
            // Condition d'Armement : Gaz bas + Yaw à fond (ex: gauche ou droite)
            // Dans ton code original : channel_3 < 1050 && channel_4 < 1050 (Yaw gauche)
            if(drone.channel_3 < 1050 && drone.channel_4 < 1050) {
                drone.current_mode = MODE_PRE_ARM;
            }
            break;

        case MODE_PRE_ARM:
            motors_stop();
            // Si on remet le Yaw au centre, on ARME
            if(drone.channel_3 < 1050 && drone.channel_4 > 1450) {
                drone.current_mode = MODE_ARMED;
                pid_reset_integral(); // Reset PID
                // Set angles de référence pour démarrage propre
                drone.angle_pitch = 0; // Ou lire acc_total_vector si dispo
                drone.angle_roll = 0;
            }
            break;

        case MODE_ARMED:
            // Moteurs tournent au ralenti ? Ou coupés si gaz à 0?
            // Selon ton code original : Gaz < 1050 = Stop
            if(drone.channel_3 < 1050) {
                motors_stop();
            } else {
                drone.current_mode = MODE_FLYING;
            }
            
            // Désarmement : Gaz bas + Yaw Droite (> 1950)
            if(drone.channel_3 < 1050 && drone.channel_4 > 1950) {
                drone.current_mode = MODE_SAFE;
            }
            break;

        case MODE_FLYING:
            pid_compute_setpoints(&drone); // Calcule Expo
            pid_compute(&drone);           // Calcule PID
            motors_mix(&drone);            // Calcule ESC
            motors_write();                // Envoie aux Moteurs
            
            // Si on coupe les gaz en vol
            if(drone.channel_3 < 1050) {
                drone.current_mode = MODE_ARMED;
            }
            break;
    }

    // 3. Gestion Loop Time (4000us)
    while(micros() - loop_timer < 4000);
    loop_timer = micros();
}