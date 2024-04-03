// Arudino Sample Code to use ADS1256 library

// Samples per second testing

// http://www.ti.com/lit/ds/symlink/ads1256.pdf

// Written by Axel Sepulveda, May 2020

#include <1256.h>
#include <SPI.h>
#define ads_restart_pin 5
float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 5; // voltage reference
ADS1256 adc(clockMHZ,vRef,false); // RESETPIN is permanently tied to 3.3v
float sensor1, sps;
long lastTime, currentTime, elapsedTime; 
int counter; 
int wrong_count=0;
void ads1256_init(void)
{
  pinMode(ads_restart_pin,OUTPUT);
  digitalWrite(ads_restart_pin,HIGH);
  Serial.println("Starting ADC");
  delay(300);
   digitalWrite(ads_restart_pin,LOW);
   delay(200);
  adc.begin(ADS1256_DRATE_500SPS,ADS1256_GAIN_1,false); 
  Serial.println("ADC Started");
  adc.setChannel(0,1);
  digitalWrite(ads_restart_pin,LOW);
}
void ads_1256_read(void)
{
   currentTime = millis();
  elapsedTime = currentTime - lastTime; 
  if (elapsedTime >= 1000){ 
    sps = counter*1.0/elapsedTime*1000;
    lastTime = currentTime; 
    counter=0;    
  }  
  adc.waitDRDY(); // wait for DRDY to go low before changing multiplexer register 
  sensor1 = adc.readCurrentChannelRaw(); // DOUT arriving here is from MUX AIN0 and AIN8 (GND)
  counter++; 
  //Serial.print("C: ");
  //Serial.print(counter);
 // Serial.print(" SR: ");
 if(!(sps>450&&sps<550))
 {
  Serial.print("wrong value");
  Serial.println(wrong_count++);
  if (wrong_count>2000)
  ESP.restart();

 }
 else
 {
  
  Serial.println(sensor1);   // print with 2 decimals
 }
}

void setup()
{
  Serial.begin(250000);
  ads1256_init();

}

void loop()
{ 
  
ads_1256_read();
  
}