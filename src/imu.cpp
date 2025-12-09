#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "imu.h"
#include "config.h"

int gyro_address;
double gyro_axis_cal[4];
// Variables temporaires pour le calcul
long acc_x, acc_y, acc_z;
int gyro_axis[4];
int temperature;

void imu_init() {
    gyro_address = EEPROM.read(32); // Lire adresse stockée

    // Setup MPU6050
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B); Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C); Wire.write(0x10);
    Wire.endTransmission();

    // FILTRE 43Hz (0x03) comme demandé
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A); Wire.write(0x03); 
    Wire.endTransmission();

    // Calibration au démarrage (2000 échantillons)
    // On fait clignoter la LED pour indiquer que ça bosse
    pinMode(12, OUTPUT);
    for (int cal_int = 0; cal_int < 2000 ; cal_int ++){
        if(cal_int % 15 == 0) digitalWrite(12, !digitalRead(12));
        
        Wire.beginTransmission(gyro_address);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(gyro_address,14);
        while(Wire.available() < 14);
        // On lit tout mais on garde que le gyro pour la calib
        acc_x = Wire.read()<<8|Wire.read();
        acc_y = Wire.read()<<8|Wire.read();
        acc_z = Wire.read()<<8|Wire.read();
        temperature = Wire.read()<<8|Wire.read();
        gyro_axis[1] = Wire.read()<<8|Wire.read();
        gyro_axis[2] = Wire.read()<<8|Wire.read();
        gyro_axis[3] = Wire.read()<<8|Wire.read();

        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];
        delay(3);
    }
    gyro_axis_cal[1] /= 2000;
    gyro_axis_cal[2] /= 2000;
    gyro_axis_cal[3] /= 2000;
    digitalWrite(12, LOW);
}

void imu_read(DroneState *drone) {
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address,14);
    while(Wire.available() < 14);
    
    acc_x = Wire.read()<<8|Wire.read();
    acc_y = Wire.read()<<8|Wire.read();
    acc_z = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();

    // Soustraire offset
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];

    // Inversions selon EEPROM (Logique User conservée)
    // Note: Pour simplifier, on suppose ici les axes standards, 
    // mais tu pourrais remettre la lecture EEPROM bits 28-30 si besoin.
    // J'utilise des variables locales pour simplifier la lecture.
    double gyro_roll = gyro_axis[1]; // Ajuster selon orientation EEPROM
    if(EEPROM.read(28) & 0b10000000) gyro_roll *= -1;
    double gyro_pitch = gyro_axis[2];
    if(EEPROM.read(29) & 0b10000000) gyro_pitch *= -1;
    double gyro_yaw = gyro_axis[3];
    if(EEPROM.read(30) & 0b10000000) gyro_yaw *= -1;

    // Filtre Logiciel (0.2 / 0.8)
    // --- Filtre PT1 Amélioré ---
    // Conversion en deg/s brute
    float gyro_roll_raw = gyro_roll * GYRO_SCALE_INV;
    float gyro_pitch_raw = gyro_pitch * GYRO_SCALE_INV;
    float gyro_yaw_raw = gyro_yaw * GYRO_SCALE_INV;

    // Application du filtre : Out = Old + Alpha * (Raw - Old)
    // Cela lisse le signal tout en minimisant le retard
    drone->gyro_roll_input = drone->gyro_roll_input + FILTER_ALPHA * (gyro_roll_raw - drone->gyro_roll_input);
    drone->gyro_pitch_input = drone->gyro_pitch_input + FILTER_ALPHA * (gyro_pitch_raw - drone->gyro_pitch_input);
    drone->gyro_yaw_input = drone->gyro_yaw_input + FILTER_ALPHA * (gyro_yaw_raw - drone->gyro_yaw_input);

    // Calculs Angles (Trigo)
    drone->angle_pitch += gyro_pitch * 0.0000611;
    drone->angle_roll += gyro_roll * 0.0000611;
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    // Accel Vector & Correction
    drone->acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    float angle_pitch_acc = 0, angle_roll_acc = 0;
    
    if(abs(acc_y) < drone->acc_total_vector){
        angle_pitch_acc = asin((float)acc_y/drone->acc_total_vector) * 57.296;
    }
    if(abs(acc_x) < drone->acc_total_vector){
        angle_roll_acc = asin((float)acc_x/drone->acc_total_vector) * -57.296;
    }
    
    // Correction Drift
    drone->angle_pitch = drone->angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    drone->angle_roll = drone->angle_roll * 0.9996 + angle_roll_acc * 0.0004;
}