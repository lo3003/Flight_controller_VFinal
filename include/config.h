#ifndef CONFIG_H
#define CONFIG_H


#define GYRO_ADDRESS 0x68
#define LOW_BATTERY_LIMIT 1000


#define PID_P_ROLL 1.5
#define PID_I_ROLL 0.005
#define PID_D_ROLL 3
#define PID_MAX_ROLL 400

// Pitch
#define PID_P_PITCH 1.5
#define PID_I_PITCH 0.005
#define PID_D_PITCH 3
#define PID_MAX_PITCH 400

// Yaw
#define PID_P_YAW 4.0
#define PID_I_YAW 0.04
#define PID_D_YAW 0.0
#define PID_MAX_YAW 400


#define GYRO_FILTER_HZ 60.0       
#define LOOP_TIME_SEC 0.004       // 4000us = 0.004s

#define FILTER_ALPHA (LOOP_TIME_SEC / (LOOP_TIME_SEC + (1.0 / (2.0 * 3.14159 * GYRO_FILTER_HZ))))

#define D_FILTER_COEFF 0.35
#define GYRO_SCALE_INV (1.0 / 65.5)
#define PID_SETPOINT_SCALE_INV (1.0 / 4.0) 
#define YAW_SETPOINT_SCALE_INV (1.0 / 3.0) 


#define MAX_THROTTLE_FLIGHT 1850  
#define GAIN_LEVEL 2              

#endif