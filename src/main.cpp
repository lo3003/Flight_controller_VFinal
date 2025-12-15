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
int debug_counter = 0;

void setup() {
    //Serial.begin(250000); 
    
    Wire.begin();
    TWBR = 12; 
    
    radio_init();
    imu_init();
    pid_init();
    motors_init();
    
    drone.current_mode = MODE_SAFE;
    


    // Attente démarrage (Gaz en bas)
    while(drone.channel_3 < 990 || drone.channel_3 > 1020) {
        radio_update(&drone);
        motors_stop();
        motors_write();
        delay(3);
    }
    
    loop_timer = micros();
}

void loop() {
    radio_update(&drone);
    imu_read(&drone);

    //Machine d'État
    switch(drone.current_mode) {
        case MODE_SAFE:
            motors_stop();
            if(drone.channel_3 < 1050 && drone.channel_4 < 1050) drone.current_mode = MODE_PRE_ARM;
            break;

        case MODE_PRE_ARM:
            motors_stop();
            if(drone.channel_3 < 1050 && drone.channel_4 > 1450) {
                drone.current_mode = MODE_ARMED;
                pid_reset_integral();
                drone.angle_pitch = 0; 
                drone.angle_roll = 0;
            }
            break;

        case MODE_ARMED:
            if(drone.channel_3 < 1050) motors_stop();
            else drone.current_mode = MODE_FLYING;
            
            if(drone.channel_3 < 1050 && drone.channel_4 > 1950) drone.current_mode = MODE_SAFE;
            break;

        case MODE_FLYING:
            pid_compute_setpoints(&drone);
            pid_compute(&drone);
            motors_mix(&drone);
            
            if(drone.channel_3 < 1050) drone.current_mode = MODE_ARMED;
            break;
    }
    motors_write();

    
    // debug_counter++;
    
    
    /*if(debug_counter >= 100) { 
        Serial.print(drone.angle_roll, 1);  // 1 chiffre après la virgule
        Serial.print(" , ");
        Serial.print(drone.angle_pitch, 1);
        Serial.print(" , ");
        Serial.print(drone.channel_3);
        Serial.print(" , ");
        Serial.println(drone.pid_output_roll, 1);
        
        debug_counter = 0;
    }*/

    if(micros() - loop_timer > 4050) digitalWrite(12, HIGH);
    else digitalWrite(12, LOW);

    while(micros() - loop_timer < 4000);
    loop_timer = micros();
}