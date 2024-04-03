/*#include<ADS1256.h>
double strain_guage_rtos=0;
extern bool extreme_taking_data;



void release_suspend_ads(void *par)
{
  while(1)
  {
    if(digitalRead(34)==LOW&&!extreme_taking_data)
   // vTaskResume(Taskh1);
    vTaskDelay(pdMS_TO_TICKS(3));

  }
}
void update_ads_dataRtos(void *par)
{

Serial.print("updated");
strain_guage_rtos=read_1256(); 
Serial.println(strain_guage_rtos);
vTaskSuspend(NULL);
 }*/