/*
  MKR Motor Carrier Pot Test
*/
#include "MKRMotorCarrier.h"

double potValue;
double potAngle;
double straightPot = 750;
double negNintyDegreePot = 225;
double potSlope = 90/ (straightPot - negNintyDegreePot);



void setup() {
  delay(10000); // Allow time to open serial monitor
  carrierSetUp();
  Serial.begin(9600); 
}

// the loop routine runs over and over again forever:
void loop() {
  potAngleFunction();
  delay(10);        // delay in between reads for stability
}


void potAngleFunction(){
  potValue = (double)analogRead(IN1);;
  potAngle = potSlope*(potValue - straightPot);
  Serial.print(potValue);
  Serial.print("  ");
  Serial.println(potAngle);
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
  
  //Take the battery status
  float batteryVoltage = (float)battery.getConverted();
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.print("V, Raw ");
  Serial.println(battery.getRaw());
}
