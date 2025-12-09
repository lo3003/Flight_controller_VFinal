#include "pid.h"
#include "config.h"

// Variables mémoires statiques
float pid_i_mem_roll, pid_last_roll_d_error, pid_roll_d_filter_old;
float pid_i_mem_pitch, pid_last_pitch_d_error, pid_pitch_d_filter_old;
float pid_i_mem_yaw, pid_last_yaw_d_error, pid_yaw_d_filter_old;

// Setpoints Base pour le filtre lissage commande
float pid_roll_setpoint_base = 0;
float pid_pitch_setpoint_base = 0;
float pid_yaw_setpoint_base = 0;

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0; pid_last_roll_d_error = 0; pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_d_error = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0; pid_last_yaw_d_error = 0; pid_yaw_d_filter_old = 0;
}

void pid_compute_setpoints(DroneState *drone) {
    // --- EXPO & RATE ROLL ---
    float input_roll = 0;
    if(drone->channel_1 > 1508) input_roll = drone->channel_1 - 1508;
    else if(drone->channel_1 < 1492) input_roll = drone->channel_1 - 1492;
    
    // Auto Level
    float level_adjust_roll = drone->angle_roll * GAIN_LEVEL;
    
    // Calcul Cube + Scale
    float target_roll = (input_roll * input_roll * input_roll) / 780000.0;
    target_roll -= level_adjust_roll;
    target_roll *= PID_SETPOINT_SCALE_INV;
    
    // Lissage
    pid_roll_setpoint_base += 0.2 * (target_roll - pid_roll_setpoint_base);
    drone->pid_setpoint_roll = pid_roll_setpoint_base;

    // --- EXPO & RATE PITCH ---
    float input_pitch = 0;
    if(drone->channel_2 > 1508) input_pitch = drone->channel_2 - 1508;
    else if(drone->channel_2 < 1492) input_pitch = drone->channel_2 - 1492;
    
    float level_adjust_pitch = drone->angle_pitch * GAIN_LEVEL;
    
    float target_pitch = (input_pitch * input_pitch * input_pitch) / 780000.0;
    target_pitch -= level_adjust_pitch;
    target_pitch *= PID_SETPOINT_SCALE_INV;
    
    pid_pitch_setpoint_base += 0.2 * (target_pitch - pid_pitch_setpoint_base);
    drone->pid_setpoint_pitch = pid_pitch_setpoint_base;

    // --- YAW ---
    // Pas d'expo complexe sur le Yaw dans ton code original
    if(drone->channel_3 > 1050){ 
        if(drone->channel_4 > 1520) drone->pid_setpoint_yaw = (drone->channel_4 - 1520) * YAW_SETPOINT_SCALE_INV;
        else if(drone->channel_4 < 1480) drone->pid_setpoint_yaw = (drone->channel_4 - 1480) * YAW_SETPOINT_SCALE_INV;
        else drone->pid_setpoint_yaw = 0;
    }
}

void pid_compute(DroneState *drone) {
    // TPA (0.0010 coeff)
    float tpa_factor = 1.0;
    if (drone->channel_3 > 1300){ // Commencer TPA plus tôt
        float reduction = (drone->channel_3 - 1300) * 0.0006; 
        tpa_factor = 1.0 - reduction;
    }

    float p_roll = PID_P_ROLL * tpa_factor;
    float d_roll = PID_D_ROLL * tpa_factor;
    float p_pitch = PID_P_PITCH * tpa_factor;
    float d_pitch = PID_D_PITCH * tpa_factor;

    // --- ROLL ---
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
    pid_i_mem_roll += PID_I_ROLL * error;
    if(pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
    else if(pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;

    float d_err_raw = pid_last_roll_d_error - drone->gyro_roll_input;
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