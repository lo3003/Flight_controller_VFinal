#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "imu.h"
#include "config.h"

int gyro_address;
double gyro_axis_cal[4];

int acc_axis[4];
int gyro_axis[4];
int temperature;

void imu_init() {
    // 1. Récupération de l'adresse I2C
    gyro_address = EEPROM.read(32); 

    // 2. Initialisation du MPU6050
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B); Wire.write(0x00); 
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B); Wire.write(0x08); 
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C); Wire.write(0x10); 
    Wire.endTransmission();
    
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A); Wire.write(0x04); 
    Wire.endTransmission();

    pinMode(12, OUTPUT);
    
    // BOUCLE DE CALIBRATION
    for (int i = 0; i < 2000 ; i++){
        if(i % 15 == 0) digitalWrite(12, !digitalRead(12));
        
        Wire.beginTransmission(gyro_address);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(gyro_address,14);
        while(Wire.available() < 14);
        
        acc_axis[1] = Wire.read()<<8|Wire.read();
        acc_axis[2] = Wire.read()<<8|Wire.read();
        acc_axis[3] = Wire.read()<<8|Wire.read();
        temperature = Wire.read()<<8|Wire.read();
        
        gyro_axis[1] = Wire.read()<<8|Wire.read();
        gyro_axis[2] = Wire.read()<<8|Wire.read();
        gyro_axis[3] = Wire.read()<<8|Wire.read();

        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];
        
        // --- CORRECTION 2 : ON PARLE AUX ESC PENDANT LA CALIB ---
        // On envoie un pulse de 1000us sur les ports 4,5,6,7 (PORTD)
        // Cela simule des gaz à 0 pour que les ESC s'initialisent
        PORTD |= B11110000;      // Pins 4-7 HIGH
        delayMicroseconds(1000); // Attente 1ms
        PORTD &= B00001111;      // Pins 4-7 LOW
        
        delay(3); // Petit délai pour atteindre ~4ms par boucle
    }
    
    // Moyenne
    gyro_axis_cal[1] /= 2000;
    gyro_axis_cal[2] /= 2000;
    gyro_axis_cal[3] /= 2000;
    
    digitalWrite(12, LOW);
}

void imu_read(DroneState *drone) {
    // ... (Le reste de imu_read reste EXACTEMENT comme avant, avec tes offsets corrigés)
    // Je remets le code complet pour être sûr
    
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address,14);
    while(Wire.available() < 14);
    
    acc_axis[1] = Wire.read()<<8|Wire.read();
    acc_axis[2] = Wire.read()<<8|Wire.read();
    acc_axis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();

    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];

    byte axis_roll_cfg = EEPROM.read(28);
    byte axis_pitch_cfg = EEPROM.read(29);
    byte axis_yaw_cfg = EEPROM.read(30);

    double gyro_roll = gyro_axis[axis_roll_cfg & 0b00000011];
    if(axis_roll_cfg & 0b10000000) gyro_roll *= -1;

    double gyro_pitch = gyro_axis[axis_pitch_cfg & 0b00000011];
    if(axis_pitch_cfg & 0b10000000) gyro_pitch *= -1;

    double gyro_yaw = gyro_axis[axis_yaw_cfg & 0b00000011];
    if(axis_yaw_cfg & 0b10000000) gyro_yaw *= -1;

    long acc_x_calc = acc_axis[axis_pitch_cfg & 0b00000011]; 
    if(axis_pitch_cfg & 0b10000000) acc_x_calc *= -1;

    long acc_y_calc = acc_axis[axis_roll_cfg & 0b00000011];
    if(axis_roll_cfg & 0b10000000) acc_y_calc *= -1;

    long acc_z_calc = acc_axis[axis_yaw_cfg & 0b00000011];
    if(axis_yaw_cfg & 0b10000000) acc_z_calc *= -1;

    // Filtre corrigé (Réactivité 0.2 / 0.8)
    float gyro_roll_raw = gyro_roll * GYRO_SCALE_INV;
    float gyro_pitch_raw = gyro_pitch * GYRO_SCALE_INV;
    float gyro_yaw_raw = gyro_yaw * GYRO_SCALE_INV;

    drone->gyro_roll_input = (drone->gyro_roll_input * 0.2) + (gyro_roll_raw * 0.8);
    drone->gyro_pitch_input = (drone->gyro_pitch_input * 0.2) + (gyro_pitch_raw * 0.8);
    drone->gyro_yaw_input = (drone->gyro_yaw_input * 0.2) + (gyro_yaw_raw * 0.8);

    drone->angle_pitch += gyro_pitch * 0.0000611;
    drone->angle_roll += gyro_roll * 0.0000611;
    
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    drone->acc_total_vector = sqrt((acc_x_calc*acc_x_calc)+(acc_y_calc*acc_y_calc)+(acc_z_calc*acc_z_calc));
    
    float angle_pitch_acc = 0, angle_roll_acc = 0;
    
    if(abs(acc_y_calc) < drone->acc_total_vector){
        angle_pitch_acc = asin((float)acc_y_calc/drone->acc_total_vector) * 57.296;
    }
    if(abs(acc_x_calc) < drone->acc_total_vector){
        angle_roll_acc = asin((float)acc_x_calc/drone->acc_total_vector) * -57.296;
    }
    
    // TES CORRECTIONS D'OFFSET (GARDE-LES)
    angle_roll_acc += 0;
    angle_pitch_acc -= 0;

    drone->angle_pitch = drone->angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    drone->angle_roll = drone->angle_roll * 0.9996 + angle_roll_acc * 0.0004;
}