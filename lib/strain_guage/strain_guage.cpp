/*#include<strain_guage.h>

double force_allowed=200;


bool object_detected_between_extremes(bool complex_flag) 
{

  double result= read_1256();
  Serial.println(result);
  if(complex_flag==real)
  return 0;
  if(  result>force_allowed)
  return 1;
  else
  return 0;

}
*/
#include<strain_guage.h>

//#include<freeRtos_ads_1220.h>
#include<ADS1256.h>
extern int32_t strain_guage_rtos;
bool extreme_taking_data=0;
double force_allowed=400;
#define free_step_limit 8
double value_at_extreme=0;
void take_value_after_two_step_of_direction_change()
{
  extreme_taking_data=1;
  
  value_at_extreme=999999999;
  while(value_at_extreme==999999999)
  {
    value_at_extreme=read_1256();
  }
  Serial.println(value_at_extreme);
  extreme_taking_data=0;
}
bool object_detected_between_extremes(bool complex_flag,uint8_t free_step,uint8_t step_count) 
{
  bool a;
  double result=1;
  // strain_guage_rtos;
  //Serial.println(result);
  if(complex_flag==imagnary)
  {
   // Serial.println("imagnary collision not allowed");
    a=0;
  }
  
  if (complex_flag==real&&free_step>9)
      {
          if(abs(abs(result)-abs(value_at_extreme))>force_allowed)
          {
            Serial.println("collision detected");
            a= 1;
          }
            else
            {
              //Serial.println("collision not detected");
                  a= 0;
            }
    }
  if (complex_flag==real&&free_step<9)
 {
   a=0;
    value_at_extreme=read_1256();   
   //Serial.print("real_free_steps");
   //Serial.println(free_step);
   }
   if (complex_flag==real&&free_step==9)
 {
   
    value_at_extreme=read_1256();   

   }
   if(abs(step_count)<3)// donot detect collision -3<stepcount<3
   a=0;


return a;

}




