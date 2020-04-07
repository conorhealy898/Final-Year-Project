/*
  IMU_Test_Rig_mk1

  This code is used to compare the roll angle output from the Arduino MKR IMU Shield
  with that of a Rotary Potentiometer.

  Arduino Hardware:
    - Arduino MKR 1010 WiFi
    - Arduino MKR Motor Carrier
    - Arduino MKR IMU Shield

  Connections:
    - Potentiometer - IN1
  
  Note when the steering potentiometer is chaned it will have to be recalibrated
  by recording the ADC value when the steering angle is straight and at -70
  degrees.

  Conor Healy                                                       8-3-2020.
*/

#include "BNO055_support.h"    
#include <Wire.h>
#include "MKRMotorCarrier.h"

double potValue;
double potAngle;
double straightPot = 782.0;
double neg70DegreePot = 362.0;
double potSlope = -70.0/ (straightPot - neg70DegreePot);

double IMUValue;
double IMUAngle;
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

unsigned long lastTime  = 0;
double T_s              = 10; //Sampling peroid in milliseconds
int sampleNumber        = 0;



void setup() {
  delay(10000); // Allow time to open serial monitor
  
  I2CSetUP();
  carrierSetUp();
  
  Serial.begin(115200); 

}

// the loop routine runs over and over again forever:
void loop() {
  if (millis() - lastTime >= T_s){
    lastTime = millis();
    sampleNumber++;
    
    IMUAngleFunction();
    potAngleFunction();
    printAngleValues();    
  }
  delayMicroseconds(1);  // Delay for stability
}


void potAngleFunction(){
  potValue = (double)analogRead(IN1);
  potAngle = potSlope*(potValue - straightPot);
}

void IMUAngleFunction(){
  bno055_read_euler_hrp(&myEulerData);      //Update Euler data into the structure
  IMUValue = double(myEulerData.r); 
  IMUAngle = IMUValue / 16.00;
}

void printAngleValues(){
  Serial.print(sampleNumber);
  Serial.print("  ");

  Serial.print(potAngle);
  Serial.print("  ");

  Serial.print(IMUAngle);
  Serial.println("  ");
}

void I2CSetUP(){
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);
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
  delay(1);
}
