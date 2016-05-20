#define KP 0.5
#define KI 1.75
#define KD 0.4
#define K_DECAY 1.007

#define OUTPUT_MAX 255
#define OUTPUT_MIN 0

#define SETPOINT 180
#define OUTPUT_PIN 3

//#include <math.h> 
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


#define FILTERSAMPLES   5
double sensSmoothArray1 [FILTERSAMPLES];   // array for holding raw sensor values for sensor1 
double sensSmoothArray2 [FILTERSAMPLES];

double digitalSmooth(double rawIn, double *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k;
  double temp, top, bottom;
  long total;
  static int i;
  static double sorted[FILTERSAMPLES];
  boolean done;

  i = (i + 1) % FILTERSAMPLES;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j=0; j<FILTERSAMPLES; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (FILTERSAMPLES - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((FILTERSAMPLES * 15)  / 100), 1); 
  top = min((((FILTERSAMPLES * 85) / 100) + 1  ), (FILTERSAMPLES - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }

  return total / k;    // divide by number of samples
}



void setup()
{
  analogWrite(3,0);
  Serial.begin(115200);
  
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();
}

int Input, Output;

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
    //Serial.print(180-Input); Serial.print("\t");
    
    Output = computePID(Input);
    //Output = digitalSmooth(Output, sensSmoothArray2);
    Serial.print(Output); Serial.print("\t");

    Serial.print("\r\n");
    analogWrite(OUTPUT_PIN, Output);
  }
  else{
    Serial.println("No ahrs");
  }
  delay(25);
}

double iTerm = 0, lastError = 0;

int computePID(int input)
{
  double error = SETPOINT - input;
  double dTerm = error - lastError;

  iTerm /= K_DECAY;
  iTerm += error/8;
  
  if(iTerm > OUTPUT_MAX) iTerm = OUTPUT_MAX;
  if(iTerm < OUTPUT_MIN) iTerm = OUTPUT_MIN;

  double Term1 = KP * error; 
  double Term2 = KI * iTerm; 
  double Term3 = KD * dTerm;
  Serial.print(Term1); Serial.print("\t");
  Serial.print(Term2); Serial.print("\t");
  Serial.print(Term3); Serial.print("\t");
  
  int output = KP * error + KI * iTerm + KD * dTerm;      
  if(output > OUTPUT_MAX) output = OUTPUT_MAX;
  if(output < OUTPUT_MIN) output = OUTPUT_MIN;
  
  lastError = error;
  return output;
}

