#ifndef CONFIG_H
#define CONFIG_H

// --- Hardware ---
#define GYRO_ADDRESS 0x68
#define LOW_BATTERY_LIMIT 1000

// --- PID Gains (Soft Tune) ---
// Roll
#define PID_P_ROLL 3.5
#define PID_I_ROLL 0.02
#define PID_D_ROLL 18.0
#define PID_MAX_ROLL 400

// Pitch
#define PID_P_PITCH 3.5
#define PID_I_PITCH 0.02
#define PID_D_PITCH 18.0
#define PID_MAX_PITCH 400

// Yaw
#define PID_P_YAW 4.0
#define PID_I_YAW 0.04
#define PID_D_YAW 0.0
#define PID_MAX_YAW 400

// --- Filtres & Optimisations ---

// --- Filtres & Optimisations ---
#define GYRO_FILTER_HZ 60.0       // Fréquence de coupure (Commencer vers 60-80Hz)
#define LOOP_TIME_SEC 0.004       // 4000us = 0.004s
// Calcul automatique du coefficient Alpha (ne pas toucher)
#define FILTER_ALPHA (LOOP_TIME_SEC / (LOOP_TIME_SEC + (1.0 / (2.0 * 3.14159 * GYRO_FILTER_HZ))))

#define D_FILTER_COEFF 0.35
#define GYRO_SCALE_INV (1.0 / 65.5)
#define PID_SETPOINT_SCALE_INV (1.0 / 4.0) // Pour Roll/Pitch
#define YAW_SETPOINT_SCALE_INV (1.0 / 3.0) // Pour Yaw

// --- Limites de Vol ---
#define MAX_THROTTLE_FLIGHT 1750  // Marge de sécurité
#define GAIN_LEVEL 5.0            // Force de l'auto-level

#endif