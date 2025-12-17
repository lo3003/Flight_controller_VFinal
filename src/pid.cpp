#include "pid.h"
#include "config.h"

// Variables mémoires statiques
float pid_i_mem_roll, pid_last_roll_d_error, pid_roll_d_filter_old;
float pid_i_mem_pitch, pid_last_pitch_d_error, pid_pitch_d_filter_old;
float pid_i_mem_yaw, pid_last_yaw_d_error, pid_yaw_d_filter_old;

#define SETPOINT_RATE_DIVIDER 4.0

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0; pid_last_roll_d_error = 0; pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_d_error = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0; pid_last_yaw_d_error = 0; pid_yaw_d_filter_old = 0;
}

void pid_compute_setpoints(DroneState *drone) {
    // --- ROLL ---
    float input_roll = 0;
    if(drone->channel_1 > 1508) input_roll = drone->channel_1 - 1508;
    else if(drone->channel_1 < 1492) input_roll = drone->channel_1 - 1492;
    
    // Auto Level (Angle P)
    float level_adjust_roll = drone->angle_roll * GAIN_LEVEL;
    
    // Calcul Direct (Sans lissage pour une réponse immédiate)
    input_roll -= level_adjust_roll;
    drone->pid_setpoint_roll = input_roll / SETPOINT_RATE_DIVIDER;

    // --- PITCH ---
    float input_pitch = 0;
    if(drone->channel_2 > 1508) input_pitch = drone->channel_2 - 1508;
    else if(drone->channel_2 < 1492) input_pitch = drone->channel_2 - 1492;
    
    float level_adjust_pitch = drone->angle_pitch * GAIN_LEVEL;
    
    input_pitch -= level_adjust_pitch;
    drone->pid_setpoint_pitch = input_pitch / SETPOINT_RATE_DIVIDER;

    // --- YAW ---
    drone->pid_setpoint_yaw = 0;
    if(drone->channel_3 > 1050){ // Si gaz actifs
        if(drone->channel_4 > 1508) 
            drone->pid_setpoint_yaw = (drone->channel_4 - 1508) / 3.0;
        else if(drone->channel_4 < 1492) 
            drone->pid_setpoint_yaw = (drone->channel_4 - 1492) / 3.0;
    }
}

void pid_compute(DroneState *drone) {
    // TPA (Atténuation des gains avec les gaz) - Désactivé par défaut
    float tpa_factor = 1.0;

    float p_roll = PID_P_ROLL * tpa_factor;
    float d_roll = PID_D_ROLL * tpa_factor;
    float p_pitch = PID_P_PITCH * tpa_factor;
    float d_pitch = PID_D_PITCH * tpa_factor;

    // --- ROLL ---
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    pid_i_mem_roll += PID_I_ROLL * error;
    // Anti-Windup
    if(pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
    else if(pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;

    float d_err_raw = pid_last_roll_d_error - drone->gyro_roll_input;
    // Filtre D
    float d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_d_error = drone->gyro_roll_input;

    drone->pid_output_roll = (p_roll * error) + pid_i_mem_roll + (d_roll * d_err_filtered);
    if(drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if(drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // --- PITCH ---
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;
    pid_i_mem_pitch += PID_I_PITCH * error;
    if(pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
    else if(pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;

    d_err_raw = pid_last_pitch_d_error - drone->gyro_pitch_input;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_d_error = drone->gyro_pitch_input;

    drone->pid_output_pitch = (p_pitch * error) + pid_i_mem_pitch + (d_pitch * d_err_filtered);
    if(drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if(drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // --- YAW ---
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;
    pid_i_mem_yaw += PID_I_YAW * error;
    if(pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
    else if(pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;

    d_err_raw = pid_last_yaw_d_error - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_d_error = drone->gyro_yaw_input;

    drone->pid_output_yaw = (PID_P_YAW * error) + pid_i_mem_yaw + (PID_D_YAW * d_err_filtered);
    if(drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if(drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}