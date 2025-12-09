#include <Arduino.h>
#include "motors.h"
#include "config.h"

int esc_1, esc_2, esc_3, esc_4;

void motors_init() {
    DDRD |= B11110000; // Configurer Pins 4-7 en sortie
    motors_stop();
}

void motors_stop() {
    esc_1 = 1000; esc_2 = 1000; esc_3 = 1000; esc_4 = 1000;
}

void motors_mix(DroneState *drone) {
    int throttle = drone->channel_3;
    
    // Bride de sécurité
    if (throttle > MAX_THROTTLE_FLIGHT) throttle = MAX_THROTTLE_FLIGHT;

    // Mixage (Divisé par 2 pour douceur au sol, selon ton code)
    // NB: J'ai retiré la logique de compensation batterie pour simplifier le .cpp
    // mais tu peux la remettre ici si tu utilises ta mesure voltage.
    
    esc_1 = throttle - (drone->pid_output_pitch * 0.5) + (drone->pid_output_roll * 0.5) - (drone->pid_output_yaw * 0.5); 
    esc_2 = throttle + (drone->pid_output_pitch * 0.5) + (drone->pid_output_roll * 0.5) + (drone->pid_output_yaw * 0.5); 
    esc_3 = throttle + (drone->pid_output_pitch * 0.5) - (drone->pid_output_roll * 0.5) - (drone->pid_output_yaw * 0.5); 
    esc_4 = throttle - (drone->pid_output_pitch * 0.5) - (drone->pid_output_roll * 0.5) + (drone->pid_output_yaw * 0.5); 

    // Ralenti minimum
    if (esc_1 < 1080) esc_1 = 1080;
    if (esc_2 < 1080) esc_2 = 1080;
    if (esc_3 < 1080) esc_3 = 1080;
    if (esc_4 < 1080) esc_4 = 1080;

    // Plafond de sécurité
    if (esc_1 > MAX_THROTTLE_FLIGHT) esc_1 = MAX_THROTTLE_FLIGHT;
    if (esc_2 > MAX_THROTTLE_FLIGHT) esc_2 = MAX_THROTTLE_FLIGHT;
    if (esc_3 > MAX_THROTTLE_FLIGHT) esc_3 = MAX_THROTTLE_FLIGHT;
    if (esc_4 > MAX_THROTTLE_FLIGHT) esc_4 = MAX_THROTTLE_FLIGHT;
}

void motors_write() {
    unsigned long loop_timer = micros();
    PORTD |= B11110000; 
    
    // Calcul des temps de fin
    unsigned long timer_1 = esc_1 + loop_timer;
    unsigned long timer_2 = esc_2 + loop_timer;
    unsigned long timer_3 = esc_3 + loop_timer;
    unsigned long timer_4 = esc_4 + loop_timer;
    
    // Boucle d'attente (Blocking)
    while(PORTD >= 16){
        unsigned long now = micros();
        if(timer_1 <= now) PORTD &= B11101111;
        if(timer_2 <= now) PORTD &= B11011111;
        if(timer_3 <= now) PORTD &= B10111111;
        if(timer_4 <= now) PORTD &= B01111111;
    }
}