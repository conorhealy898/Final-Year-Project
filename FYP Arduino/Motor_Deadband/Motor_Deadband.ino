/*
  Motor_Deadband

  This code is used to determine what duty cycle is required to overcome
  the deadband in a motor.

  Connections:
    - Motor - M4.

  Conor Healy                                                  19-3-2020.
*/

// Libraries
#include "BNO055_support.h"    
#include <Wire.h>
#include "MKRMotorCarrier.h"

double batteryVoltage;
double delayTime = 2000;

void setup() {
  carrierSetUp();
  Serial.begin(115200); 
  
  delay(10000); // Allow time to open the serial monitor
  batteryCheck();
}

// the loop routine runs over and over again forever:
void loop() {
  for(int duty = 10; duty < 20; duty++){
    M4.setDuty(duty);
    Serial.println(duty);
    delay(delayTime);  // Delay for stability
  }
}

void batteryCheck(){
  batteryVoltage = (double)battery.getConverted();
  
  if (batteryVoltage < 11) 
  {
    Serial.println(" ");
    Serial.println("WARNING: LOW BATTERY");
    Serial.println("ALL SYSTEMS DOWN");
    M1.setDuty(0);
    M2.setDuty(0);
    M3.setDuty(0);
    M4.setDuty(0);
    
    while (batteryVoltage < 11) 
    {
      batteryVoltage = (double)battery.getConverted();
      Serial.println("Battery Still Too Low");
      delay(100);
    }
  }
}

void carrierSetUp() {
  //Establishing the communication with the motor shield
  if (controller.begin()) 
    {
      Serial.print("MKR Motor Shield connected, firmware version ");
      Serial.println(controller.getFWVersion());
    } 
  else 
    {
      Serial.println("Couldn't connect! Is the red led blinking? You may need to update the firmware with FWUpdater sketch");
      while (1);
    }

  // Reboot the motor controller; brings every value back to default
  Serial.println("reboot");
  controller.reboot();
  delay(500);

  // Set all duty cycles to zero
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);

  // Reset the encoder internal counter to zero (can be set to any initial value)
  Serial.println("reset counters");
  encoder1.resetCounter(0);
  encoder2.resetCounter(0); 
  
  //Take the battery status
  batteryVoltage = (double)battery.getConverted();
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.print("V, Raw ");
  Serial.println(battery.getRaw());
  delay(1);
}
