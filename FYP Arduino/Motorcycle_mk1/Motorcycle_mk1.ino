/*
  Motorcycle_mk1
  
  This code is used to control the self-balancing motorcycle model uses in the
  "Active Stabilisation of Motorcycle Roll Dynamics at Low Speeds" final year
  project.

  Arduino Hardware:
    - Arduino MKR 1010 WiFi
    - Arduino MKR Motor Carrier
    - Arduino MKR IMU Shield

  Connections:
    - Steering Motor - M1
    - Steering Potentiometer - IN1
    - Drive Motor - M4
    - Drive Motor Encoder - encoder2

  Note when the steering potentiometer is chaned it will have to be recalibrated
  by recording the ADC value when the steering angle is straight and at -70
  degrees.

  Conor Healy                                                     25 - 3 - 2020.
    
*/

// Libraries
#include "BNO055_support.h"    
#include <Wire.h>
#include "MKRMotorCarrier.h"

// Unit Conversions
double pi      = 3.1416;
double deg2Rad = pi / 180.0;
double rad2Deg = 1.0 / deg2Rad;

// Roll System
struct bno055_t myBNO;
unsigned char accelCalibStatus  = 0;  //Variable to hold the calibration status of the Accelerometer
unsigned char magCalibStatus    = 0;  //Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus   = 0;  //Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus    = 0;  //Variable to hold the calibration status of the System (BNO055's MCU)
struct bno055_euler myEulerData;      //Structure to hold the Euler data
double IMUValue;
double IMUOffset                = 0;

// Steering System
double potValue;
double straightPot          = 778.0;
double negSeventyDegreePot  = 381.0;
double potSlope             = -70.0/ (straightPot - negSeventyDegreePot);

// Drive System
int encoderRawCount;
int encoderCountsPerSec;
double omega_drive_motor;
double wheelRadius    = 60e-3;
double driveGearRatio = 100.0;
double radiusGear = wheelRadius / driveGearRatio;

// Timing
unsigned long lastSampleTime    = 0;
unsigned long T_sample          = 10;       //Sampling peroid in milliseconds
unsigned long lastSetpointTime  = 0;
unsigned long T_setpoint        = 20;
int sampleNumber                = 0;
unsigned long lastIMUTime       = 0;

// Roll P Controller Paramaters
double KcRollP = 20.0;

// Roll Phase Lead Controller Paramaters
double KcRollPL   = -7.780;
double zeroRollPL = 0.9260;
double poleRollPL = 0.9866;

// Roll DPZ Controller Parameters
double KcRollDPZ;
double zeroRollDPZ_1;
double zeroRollDPZ_2;
double poleRollDPZ_1;
double poleRollDPZ_2;

double zeroRollDPZSum      = zeroRollDPZ_1 + zeroRollDPZ_2;
double zeroRollDPZProduct  = zeroRollDPZ_1 * zeroRollDPZ_2;
double poleRollDPZSum      = poleRollDPZ_1 + poleRollDPZ_2;
double poleRollDPZProduct  = poleRollDPZ_1 * poleRollDPZ_2;

// Common Roll Controller Paramaters
double errorRoll_K    = 0;
double errorRoll_Km1  = 0;
double errorRoll_Km2  = 0;
double errorRollDeg_K = 0;

double deltaReqDeg_K  = 0;
double deltaReq_K     = deltaReqDeg_K * deg2Rad;
double deltaReq_Km1   = 0;
double deltaReq_Km2   = 0;
bool deltaReqPlus     = 1;

// Steering P Controller Parameters
double KcSteeringP  = 2.0;

// Steering Phase Lead Controller Parameters
double KcSteeringPL = 8.1788;
double zeroSteering = 0.8994;
double poleSteering = 0.9930;

double errorSteering_K    = 0;
double errorSteering_Km1  = 0;
double errorSteeringDeg_K = 0;

double VasReq_K   = 0;
double VasReq_Km1 = 0;

// Drive P Controller Parameters
double KcDrive      = 25.0;
double errorDrive_K = 0;
double VadReq_K     = 0;  //7.0;

// Inputs
double v_xReq_K     = 0.0;
double phiReqDeg_K  = 0.0;
double phiReq_K     = phiReqDeg_K * deg2Rad;


// Outputs
double v_x_K              = 0;
double deltaDeg_K         = 0;
double delta_K            = 0;
double phiDeg_K           = 0;
double phi_K              = 0;
int driveDuty             = 0;
int driveDeadBandDuty     = 15;
int steeringDuty          = 0;
int steeringDeadBandDuty  = 5;
double Vas_K              = 0;
double Vad_K              = 0;

// Supply and Saturation
double VSupply      = 11.1;
int maxDuty         = 100;
double deltaMaxDeg  = 5.0;
double deltaMax     = deltaMaxDeg * deg2Rad;
double batteryVoltage;

void setup() {
  I2CSetUP();
  carrierSetUp();
  Serial.begin(115200); 

  batteryCheck();
  IMUCalibration();
  delay(10000);
}

// the loop routine runs over and over again forever:
void loop() {
  if (millis() - lastSampleTime >= T_sample){
    lastSampleTime = millis();
    sampleNumber++;
    
    speedFunction();
    rollAngleFunction();
    steeringAngleFunction();
 
    //pRoll();
    phaseLeadRoll();
    //dpzRoll();

    //pSteering();
    phaseLeadSteering();

    pDrive();
    
    updateDutyCycle();
    printOutputs();
  }

  /*
  if (millis() - lastSetpointTime >= T_setpoint){
    lastSetpointTime = millis();
    deltaReqFunction();
  }
  */
  
  delayMicroseconds(5);  // Delay for stability
}

void pSteering(){
  errorSteering_K     = deltaReq_K - delta_K;
  errorSteeringDeg_K  = errorSteering_K * rad2Deg;
  
  VasReq_K = KcSteeringP * errorSteering_K;
  VasReq_K = doubleSaturationRemoval(VasReq_K, VSupply);
  
  delayMicroseconds(5);
}

void phaseLeadSteering(){
  errorSteering_K     = deltaReq_K - delta_K;
  errorSteeringDeg_K  = errorSteering_K * rad2Deg;
  
  VasReq_K = KcSteeringPL * (errorSteering_K - zeroSteering * errorSteering_Km1) + poleSteering * VasReq_Km1; 
  VasReq_K = doubleSaturationRemoval(VasReq_K, VSupply);
  
  errorSteering_Km1 = errorSteering_K;
  VasReq_Km1        = VasReq_K;
  
  delayMicroseconds(5);
}

void pRoll(){
  errorRoll_K     = phiReq_K - phi_K;
  errorRollDeg_K  = errorRoll_K * rad2Deg;
  
  deltaReq_K    = KcRollP * errorRoll_K;
  deltaReq_K    = doubleSaturationRemoval(deltaReq_K, deltaMax);
  deltaReqDeg_K = deltaReq_K * rad2Deg;
  
  delayMicroseconds(5);
}

void phaseLeadRoll(){
  errorRoll_K     = phiReq_K - phi_K;
  errorRollDeg_K  = errorRoll_K * rad2Deg;
  
  deltaReq_K    = KcRollPL * (errorRoll_K - zeroRollPL * errorRoll_Km1) + poleRollPL * deltaReq_Km1;
  //deltaReq_K    = -deltaReq_K;
  deltaReq_K    = doubleSaturationRemoval(deltaReq_K, deltaMax);
  deltaReqDeg_K = deltaReq_K * rad2Deg;
  
  errorRoll_Km1 = errorRoll_K;
  deltaReq_Km1  = deltaReq_K;
  delayMicroseconds(5);
}

void dpzRoll(){
  errorRoll_K     = phiReq_K - phi_K;
  errorRollDeg_K  = errorRoll_K * rad2Deg;
  
  deltaReq_K    = KcRollDPZ * (errorRoll_K - (zeroRollDPZSum)*errorRoll_Km1 + zeroRollDPZProduct*errorRoll_Km2) + poleRollDPZSum*deltaReq_Km1 + poleRollDPZProduct*deltaReq_Km2;
  //deltaReq_K    = -deltaReq_K;
  deltaReq_K    = doubleSaturationRemoval(deltaReq_K, deltaMax);
  deltaReqDeg_K = deltaReq_K * rad2Deg;
  
  errorRoll_Km1 = errorRoll_K;
  errorRoll_Km2 = errorRoll_Km1;
  deltaReq_Km1  = deltaReq_K;
  deltaReq_Km2  = deltaReq_Km1;
  
  delayMicroseconds(5);
}

void pDrive(){
  errorDrive_K = v_xReq_K - v_x_K;
  
  VadReq_K = KcDrive * errorDrive_K;
  VadReq_K = doubleSaturationRemoval(VadReq_K, VSupply);
  
  delayMicroseconds(5);
}

void updateDutyCycle(){
  driveDuty     = (int)(maxDuty * VadReq_K / VSupply);
  if(driveDuty > 0){
    driveDuty += driveDeadBandDuty;
  }
  if(driveDuty < 0){
    driveDuty -= driveDeadBandDuty;
  }
  intSaturationRemoval(driveDuty, maxDuty);
  Vad_K         = (double)driveDuty/ (double)maxDuty * VSupply;

  
  steeringDuty  = (int)(maxDuty * VasReq_K / VSupply);
  if(steeringDuty > 0){
    steeringDuty += steeringDeadBandDuty;
  }
  if(steeringDuty < 0){
    steeringDuty -= steeringDeadBandDuty;
  }
  intSaturationRemoval(steeringDuty, maxDuty);
  Vas_K         = (double)steeringDuty/ (double)maxDuty * VSupply;

  M4.setDuty(driveDuty);
  M1.setDuty(steeringDuty);
  delayMicroseconds(5);
}

int intSaturationRemoval(int sig, int limit){
  if (sig > limit){
    sig = limit;
  }

  if (sig < -limit){
    sig = -limit;
  }

  return sig;
}

double doubleSaturationRemoval(double sig, double limit){
  if (sig > limit){
    sig = limit;
  }

  if (sig < -limit){
    sig = -limit;
  }

  return sig;
}


void speedFunction(){
  encoderRawCount = encoder2.getRawCount();
  encoderCountsPerSec = encoder2.getCountPerSecond();
  
  omega_drive_motor = 2.0 * pi * (double)encoderCountsPerSec;
  v_x_K = radiusGear * omega_drive_motor; 
}

void steeringAngleFunction(){
  potValue    = (double)analogRead(IN1);
  
  deltaDeg_K  = potSlope*(potValue - straightPot);
  delta_K     = deltaDeg_K * deg2Rad;
}

void rollAngleFunction(){
  bno055_read_euler_hrp(&myEulerData);      //Update Euler data into the structure
  IMUValue = (double)(myEulerData.r);
   
  phiDeg_K  = IMUOffset +  (IMUValue / 16.00);
  phi_K     = phiDeg_K * deg2Rad;
}

void deltaReqFunction(){
  if(deltaReqPlus == 1){
    deltaReq_K += 1;

    if(deltaReq_K >= deltaMax){
      deltaReq_K = deltaMax;
      deltaReqPlus = 0;
    }
  }
  else{
    deltaReq_K -= 1;

    if(deltaReq_K <= -deltaMax){
      deltaReq_K = -deltaMax;
      deltaReqPlus = 1;
    }
  }
}
  

void printOutputs(){
    Serial.print(sampleNumber);
    Serial.print("  ");

    Serial.print(phiReqDeg_K);
    Serial.print("  ");
    Serial.print(phiDeg_K);
    Serial.print("  ");
    Serial.print(errorRollDeg_K);
    Serial.print("     ");

    Serial.print(deltaReqDeg_K);
    Serial.print("  ");
    Serial.print(deltaDeg_K);
    Serial.print("  ");
    Serial.print(errorSteeringDeg_K);
    Serial.print("     ");

    Serial.print(v_xReq_K);
    Serial.print("  ");
    Serial.print(v_x_K);
    Serial.print("  ");
    Serial.print(errorDrive_K);
    Serial.print("     ");

    Serial.print(VasReq_K);
    Serial.print("  ");
    Serial.print(Vas_K);
    Serial.print("      ");
    
    Serial.print(VadReq_K);
    Serial.print("  ");
    Serial.println(Vad_K);
    
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

void IMUCalibration(){
  while((magCalibStatus != 3) || (gyroCalibStatus != 3) || (sysCalibStatus != 3)){
    if((millis() - lastIMUTime) >= 200) //To read calibration status at 5Hz without using additional timers
    {
      lastIMUTime = millis();
    
      Serial.print("Time Stamp: ");     //To read out the Time Stamp
      Serial.println(lastIMUTime);
    
      bno055_get_accelcalib_status(&accelCalibStatus);
      Serial.print("Accelerometer Calibration Status: ");   //To read out the Accelerometer Calibration Status (0-3)
      Serial.println(accelCalibStatus);
    
      bno055_get_magcalib_status(&magCalibStatus);
      Serial.print("Magnetometer Calibration Status: ");    //To read out the Magnetometer Calibration Status (0-3)
      Serial.println(magCalibStatus);
    
      bno055_get_magcalib_status(&gyroCalibStatus);
      Serial.print("Gyroscope Calibration Status: ");     //To read out the Gyroscope Calibration Status (0-3)
      Serial.println(gyroCalibStatus);
    
      bno055_get_syscalib_status(&sysCalibStatus);
      Serial.print("System Calibration Status: ");      //To read out the Magnetometer Calibration Status (0-3)
      Serial.println(sysCalibStatus);
    
      Serial.println();                   //To separate between packets
    }
  }
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
