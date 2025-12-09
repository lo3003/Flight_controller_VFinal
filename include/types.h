#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// --- MACHINE D'ÉTAT (FSM) ---
typedef enum {
    MODE_SAFE,      // Moteurs coupés (Disarmed)
    MODE_PRE_ARM,   // Étape intermédiaire (Manches dans les coins)
    MODE_ARMED,     // Moteurs tournent au ralenti (Idle)
    MODE_FLYING     // Gaz montés, PID actifs
} FlightMode;

// --- OBJET PRINCIPAL ---
typedef struct {
    // État du Cerveau
    FlightMode current_mode;

    // Radio Inputs (1000-2000us)
    int channel_1; // Roll
    int channel_2; // Pitch
    int channel_3; // Throttle
    int channel_4; // Yaw

    // Capteurs (IMU)
    float gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
    float angle_roll;
    float angle_pitch;
    float acc_total_vector; // Pour calcul angle

    // PID
    float pid_setpoint_roll;
    float pid_setpoint_pitch;
    float pid_setpoint_yaw;
    
    float pid_output_roll;
    float pid_output_pitch;
    float pid_output_yaw;

    // Système
    int battery_voltage;
    
} DroneState;

#endif