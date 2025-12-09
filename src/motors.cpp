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
    
    // 1. Clamp du Throttle commande
    if (throttle > MAX_THROTTLE_FLIGHT) throttle = MAX_THROTTLE_FLIGHT;

    // 2. Calcul du Mixage Brut (Sans limite pour l'instant)
    // On calcule en variables locales pour pouvoir analyser les dépassements
    int esc_1_calc = throttle - (drone->pid_output_pitch * 0.5) + (drone->pid_output_roll * 0.5) - (drone->pid_output_yaw * 0.5); 
    int esc_2_calc = throttle + (drone->pid_output_pitch * 0.5) + (drone->pid_output_roll * 0.5) + (drone->pid_output_yaw * 0.5); 
    int esc_3_calc = throttle + (drone->pid_output_pitch * 0.5) - (drone->pid_output_roll * 0.5) - (drone->pid_output_yaw * 0.5); 
    int esc_4_calc = throttle - (drone->pid_output_pitch * 0.5) - (drone->pid_output_roll * 0.5) + (drone->pid_output_yaw * 0.5); 

    // 3. Gestion de la Saturation Max (Airmode logic)
    // On cherche la commande moteur la plus élevée
    int max_esc = esc_1_calc;
    if(esc_2_calc > max_esc) max_esc = esc_2_calc;
    if(esc_3_calc > max_esc) max_esc = esc_3_calc;
    if(esc_4_calc > max_esc) max_esc = esc_4_calc;

    // Si un moteur demande plus que le max autorisé, on réduit TOUT le monde
    // Cela préserve le différentiel (PID) nécessaire pour tourner
    if(max_esc > MAX_THROTTLE_FLIGHT) {
        int overshoot = max_esc - MAX_THROTTLE_FLIGHT;
        esc_1_calc -= overshoot;
        esc_2_calc -= overshoot;
        esc_3_calc -= overshoot;
        esc_4_calc -= overshoot;
    }

    // 4. Assignation et Sécurité Basse
    esc_1 = esc_1_calc;
    esc_2 = esc_2_calc;
    esc_3 = esc_3_calc;
    esc_4 = esc_4_calc;

    // Ralenti minimum (Idle) - Empêche les moteurs de s'arrêter en vol
    if (esc_1 < 1080) esc_1 = 1080;
    if (esc_2 < 1080) esc_2 = 1080;
    if (esc_3 < 1080) esc_3 = 1080;
    if (esc_4 < 1080) esc_4 = 1080;
    
    // Sécurité finale (au cas où le calcul d'overshoot aurait raté un cas limite)
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