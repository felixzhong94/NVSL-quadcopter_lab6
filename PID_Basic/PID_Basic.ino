/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include <math.h> 

#include <PID_v1.h>
#define PIN_OUTPUT 3

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>

// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  analogWrite(3,0);
  Serial.begin(115200);
  Serial.println(F("Adafruit LSM9DS0 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();
  
  
  
  //initialize the variables we're linked to
  //Input = analogRead(PIN_INPUT);
  Setpoint = 180;

  //turn the PID on
  myPID.SetOutputLimits(-128,128);
  myPID.SetMode(AUTOMATIC);
}

int x = 0;
void loop()
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(F("Orientation: "));
    
    Input = orientation.roll;
    Input = (Input < 0)?-Input:380-Input;
    
    //Input = (Input < 0)? 360+Input: Input;
    //Serial.print(roll);
    //Serial.print(F(" "));
    //Serial.print(orientation.pitch);
    //Serial.print(F(" "));
    //Serial.print(orientation.heading);
    //Serial.println(F(""));
    /*if(roll > 0){
      analogWrite(3,1 55);
      Serial.println(" - ON");
    }else{
      analogWrite(3,0);
      Serial.println(" - OFF");
    }*/
  
    myPID.Compute();
    
    
    
    //Output = (Output > 255)?255:Output;

    double rad = Input * (0.01745329251);
    double offset = -60 * cos(rad);  
    if(offset < 0) offset = 0;    
    
    //if(Output < 20) Output = 20;
    
    
    Serial.print(Input);
    Serial.print(" -> ");
    //Serial.print(offset); 
    Serial.print(x);
    Serial.print(" -> ");
    Serial.print(Output); 
    Serial.print(" -> ");
    Serial.println(Output+offset); 

    double write_val = Output+offset;
    if(write_val > 200) write_val = 200;
    if(write_val < 0) write_val = 0;
    //analogWrite(PIN_OUTPUT, Output+offset);
    
    while(Input > 180);
    
    
  }
  else{
    Serial.println("No ahrs");
  
  
  
  }
  if(++x > 255) x= 0;
  analogWrite(PIN_OUTPUT, x);
  
  delay(100);

}


