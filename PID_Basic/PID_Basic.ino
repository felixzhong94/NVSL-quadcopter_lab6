#define filterSamples   25              // filterSamples should  be an odd number, no smaller than 3

//Specify the links and initial tuning parameters
double Kp=1, Ki=1, Kd=.2;

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



double sensSmoothArray1 [filterSamples];   // array for holding raw sensor values for sensor1 
double sensSmoothArray2 [filterSamples];

double digitalSmooth(double rawIn, double *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k;
  double temp, top, bottom;
  long total;
  static int i;
  static double sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }

  return total / k;    // divide by number of samples
}




//Define Variables we'll be connecting to
double Setpoint, Input, Output;


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
  myPID.SetOutputLimits(0,255);
  myPID.SetSampleTime(10);
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

    Input = digitalSmooth(Input, sensSmoothArray1);
    
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
  
    Serial.print(180-Input); Serial.print("\t");
    myPID.Compute();
    
    
    
    //Output = (Output > 255)?255:Output;

    //double rad = Input * (0.01745329251);
    //double offset = -60 * cos(rad);  
    //if(offset < 0) offset = 0;    
    
    //Serial.print(offset); 
    //Serial.print("\t");
    //Serial.print(Output); Serial.print("\t");


    //double write_val = Output;//+offset;
    double write_val = Output;//= digitalSmooth(Output, sensSmoothArray2);
    if(write_val > 255) write_val = 255;
    if(write_val < 0) write_val = 0;
    analogWrite(PIN_OUTPUT, write_val);
        
    //Serial.println(write_val); Serial.print("\t");
    Serial.print("\r\n");
  }
  else{
    Serial.println("No ahrs");
  }
  
  delay(5);

}


