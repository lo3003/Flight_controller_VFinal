#include <Arduino.h>
#include <EEPROM.h>
#include "radio.h"

// Variables volatiles pour l'ISR
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
byte eeprom_data[36]; 

// Variables internes pour la conversion
int receiver_input[5]; 


int convert_receiver_channel(byte function) {
    byte channel, reverse;
    int low, center, high, actual, difference;

    channel = eeprom_data[function + 23] & 0b00000111;
    if(eeprom_data[function + 23] & 0b10000000) reverse = 1; else reverse = 0;

    actual = receiver_input[channel];
    low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
    center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
    high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

    if(actual < center){
        if(actual < low) actual = low;
        difference = ((long)(center - actual) * (long)500) / (center - low);
        if(reverse == 1) return 1500 + difference; else return 1500 - difference;
    } else if(actual > center){
        if(actual > high) actual = high;
        difference = ((long)(actual - center) * (long)500) / (high - center);
        if(reverse == 1) return 1500 - difference; else return 1500 + difference;
    } else return 1500;
}

void radio_init() {
    // Interruptions PCINT0 (Pins 8, 9, 10, 11)
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT1);
    PCMSK0 |= (1 << PCINT2);
    PCMSK0 |= (1 << PCINT3);

    
    for(int start = 0; start <= 35; start++) eeprom_data[start] = EEPROM.read(start);
}

void radio_update(DroneState *drone) {
    drone->channel_1 = convert_receiver_channel(1);
    drone->channel_2 = convert_receiver_channel(2);
    drone->channel_3 = convert_receiver_channel(3);
    drone->channel_4 = convert_receiver_channel(4);
}


ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1
  if(PINB & B00000001){
    if(last_channel_1 == 0){ last_channel_1 = 1; timer_1 = current_time; }
  } else if(last_channel_1 == 1){ last_channel_1 = 0; receiver_input[1] = current_time - timer_1; }
  //Channel 2
  if(PINB & B00000010 ){
    if(last_channel_2 == 0){ last_channel_2 = 1; timer_2 = current_time; }
  } else if(last_channel_2 == 1){ last_channel_2 = 0; receiver_input[2] = current_time - timer_2; }
  //Channel 3
  if(PINB & B00000100 ){
    if(last_channel_3 == 0){ last_channel_3 = 1; timer_3 = current_time; }
  } else if(last_channel_3 == 1){ last_channel_3 = 0; receiver_input[3] = current_time - timer_3; }
  //Channel 4
  if(PINB & B00001000 ){
    if(last_channel_4 == 0){ last_channel_4 = 1; timer_4 = current_time; }
  } else if(last_channel_4 == 1){ last_channel_4 = 0; receiver_input[4] = current_time - timer_4; }
}