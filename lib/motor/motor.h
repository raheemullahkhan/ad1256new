#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#define ADS1220_CS_PIN    5 // chip select pin
#define ADS1220_DRDY_PIN  34 // data ready pin 



#define DIRECTION_PIN 0
#define ENABLE_PIN 2
#define Real_pulse_pin 15
#define imagnary_pulse_pin 33 // this pin doesnot exist in reality 

#define pulse_per_rev 3200
#define pulse_in_one_step (pulse_per_rev)/64  // min step is rev/64

void enable_motor();
void disable_motor();
void generate_steps(int number_of_steps, int pulseDuration);
void changeDirection();
void set_down_direction();
void set_up_direction();
void motor_init();

#endif // MOTOR_CONTROL_H