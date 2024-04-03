/*#include<motor.h>
#include <Arduino.h>
#include<freertos/FreeRTOS.h>
#include<freertos/task.h>
#include<strain_guage.h>
#define real 1
#define imagnary 0
#define imagnary 1
#define imagnary_first_time 1
#define imagnary_second_time 0
bool imagnary_entered=imagnary_first_time;
bool complex_flag=real;
bool direction = 0;
bool imagnary_first_entered_direction;
bool imagnary_direction=direction;
uint32_t imagnary_step_count=0;
void cheack_first_time_entered()
{
if (imagnary_entered==imagnary_first_time)
        imagnary==imagnary_second_time;
}
bool imagnary_direction_reversed()
{
if(imagnary_entered==imagnary_first_time)
   {
     imagnary_direction=direction;
     imagnary_first_entered_direction=direction;
    return 0;
   }
if(imagnary_direction!=imagnary_first_entered_direction)
    return 1;
else 
    return 0;

}

static void real_move_one_step(int pulseDuration)
{
    complex_flag=real;
    for (int i = 0; i < pulse_in_one_step; ++i) 
    {
        digitalWrite(Real_pulse_pin, HIGH);
         delayMicroseconds (pulseDuration);
        digitalWrite(Real_pulse_pin, LOW);
        delayMicroseconds(pulseDuration);
    }

}
static void imagnary_move_one_step(int pulseDuration)
{
    complex_flag=imagnary;
    if (imagnary_direction_reversed())
        imagnary_step_count--;
    else 
       {
         imagnary_step_count++;
         imagnary_entered=imagnary_second_time;
       }
    if (imagnary_step_count==0)//imagnary round completed 
       {
        complex_flag=0;
        imagnary_entered=imagnary_first_time;
       }



    for (int i = 0; i < pulse_in_one_step; ++i) 
    {
        digitalWrite(imagnary_pulse_pin, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(imagnary_pulse_pin, LOW);
        delayMicroseconds(pulseDuration);
    }

}

void generate_steps(int number_of_steps, int pulseDuration)
 {
   for(int j=0;j<number_of_steps;j++)
   {
    if(object_detected_between_extremes(complex_flag))//detect collision in real motion not during imagnary motion
     { 
        cheack_first_time_entered();
        imagnary_move_one_step(pulseDuration);
     }
    else
      {
        real_move_one_step(pulseDuration); 
      }
   }
}

void changeDirection() {
    direction = !direction;
    digitalWrite(DIRECTION_PIN, direction);  // Toggle direction
}

void set_down_direction() {
    digitalWrite(DIRECTION_PIN, LOW);
}

void set_up_direction() {
    digitalWrite(DIRECTION_PIN, HIGH);
}
void motor_init() {
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(Real_pulse_pin, OUTPUT);
    digitalWrite(DIRECTION_PIN, HIGH);  // Assuming HIGH is the initial direction
}
void enable_motor() {
    digitalWrite(ENABLE_PIN, HIGH);
}
void disable_motor() {
    digitalWrite(ENABLE_PIN, LOW);
}*/











#include<motor.h>
#include <Arduino.h>
#include<freertos/FreeRTOS.h>
#include<freertos/task.h>
#include<strain_guage.h>
#define real 1
#define imagnary 0
#define upwarde 0
#define downword 1
#define imagnary_first_time 1
#define imagnary_second_time 0
#define reversed 1
#define not_reversed 0
uint64_t step_count=0;
int steps_after_direction_change;
uint64_t step_at_time_of_imagnary_entered=0;
uint8_t real_step_after_direction_change=20;
int real_free_step=500;
bool imagnary_entered=imagnary_first_time;
bool complex_flag=real;
bool direction = 0;
bool imagnary_first_entered_direction;
bool imagnary_direction=direction;
bool imagnary_direction_change_test=not_reversed;
uint64_t imagnary_step_count=0;


void cheack_imagnary_direction_reversed()
{
   // Serial.print("direction");
    //println(direction);
if(imagnary_entered==imagnary_first_time)
   {
     imagnary_direction=direction;
     imagnary_first_entered_direction=direction;
     imagnary_direction_change_test=not_reversed;
     Serial.println("first_direction_noted");
   }

if(imagnary_direction!=imagnary_first_entered_direction&&imagnary_entered==imagnary_second_time)
    {
        imagnary_direction=direction;
        imagnary_direction_change_test=reversed;
        Serial.println("reversed");
    }
 if(imagnary_direction==imagnary_first_entered_direction&&imagnary_entered==imagnary_second_time)
    {
        imagnary_direction=direction;
        imagnary_direction_change_test=not_reversed;
        Serial.println("same");
    }
}



void cheack_first_time_entered()
{
if (imagnary_entered==imagnary_first_time)
    {
        Serial.println("steps at time of imagnary_entered");
        step_at_time_of_imagnary_entered=step_count;
        Serial.println(step_at_time_of_imagnary_entered);
        imagnary==imagnary_second_time;
    }
}


static void real_move_one_step(int pulseDuration)
{
    real_free_step++;
    if (real_free_step>100)
    real_free_step=50;
    steps_after_direction_change++;
    if(steps_after_direction_change==2)
    {
        Serial.println("note extreme value");  
       take_value_after_two_step_of_direction_change();
    }
    for (int i = 0; i < pulse_in_one_step; ++i) 
    {
        digitalWrite(Real_pulse_pin, HIGH);
         delayMicroseconds (pulseDuration);
        digitalWrite(Real_pulse_pin, LOW);
        delayMicroseconds(pulseDuration);
    }
    if(direction==upwarde)
    step_count++;
if (direction==downword)
    step_count--;
//Serial.print("real steps count");
 //  Serial.println(step_count);


}

static void imagnary_move_one_step(int pulseDuration)
{
    for (int i = 0; i < pulse_in_one_step; ++i) 
    {
        digitalWrite(imagnary_pulse_pin, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(imagnary_pulse_pin, LOW);
        delayMicroseconds(pulseDuration);
    }
if(direction==upwarde)
    step_count++;
if (direction==downword)
    step_count--;
   // Serial.print("imagnary steps count");
    //Serial.println(step_count);
    if (imagnary_direction_change_test==reversed)
       { imagnary_step_count--;
         // Serial.print("imagnary_count");
         Serial.println(imagnary_step_count);
       }
    else 
       {
         imagnary_step_count++;
         //Serial.print("imagnary_count");
         //Serial.println(imagnary_step_count);
         imagnary_entered=imagnary_second_time;
       }

             Serial.print("step count in imagnary portion");
        Serial.println(step_count);
        Serial.println(reversed);
    if (step_count==step_at_time_of_imagnary_entered)//imagnary round completed 
       {
        complex_flag=real;
        Serial.print("steps at time of imagnary_leaving");
        Serial.println(step_count);
        imagnary_entered=imagnary_first_time;
        real_free_step=0;
 
       }

}

void update_flags(void)
{
    if(object_detected_between_extremes(complex_flag,real_free_step,step_count))//detect collision in real motion not during imagnary motion 
    {
        complex_flag=imagnary;
        steps_after_direction_change=0;
        
        
        
    }
    
}
void generate_steps(int number_of_steps, int pulseDuration)
 {
   for(int j=0;j<number_of_steps;j++)
   {
    update_flags();
    if(complex_flag==imagnary)
    {
         cheack_imagnary_direction_reversed();
          cheack_first_time_entered();
          imagnary_move_one_step(pulseDuration);
    }
    else
    real_move_one_step(pulseDuration);
   }
}

void changeDirection() {

    direction = !direction;
    steps_after_direction_change=0;
    if(complex_flag==real)
    steps_after_direction_change=0;
    digitalWrite(DIRECTION_PIN, direction);  // Toggle direction
}

void set_down_direction() {
    digitalWrite(DIRECTION_PIN, LOW);
}

void set_up_direction() {
    digitalWrite(DIRECTION_PIN, HIGH);
}
void motor_init() {
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(Real_pulse_pin, OUTPUT);
    digitalWrite(DIRECTION_PIN, HIGH);  // Assuming HIGH is the initial direction
}
void enable_motor() {
    digitalWrite(ENABLE_PIN, HIGH);
}
void disable_motor() {
    digitalWrite(ENABLE_PIN, LOW);
}