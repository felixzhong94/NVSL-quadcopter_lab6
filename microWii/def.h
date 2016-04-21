#ifndef DEF_H_
#define DEF_H_


/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega128RFA1__)
  #define PROMINI
#endif


/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/
#define SERVO_RATES      {30,30,100,100,100,100,100,100}

#if defined(DYNBALANCE)
  #define DYNBAL 1
#else
  #define DYNBAL 0
#endif
#if defined(FLAPS)
  #define FLAP 1
#else
  #define FLAP 0
#endif

#if defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
  #define NUMBER_MOTOR     4
#endif

/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN D0&D1 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(BMA280) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(LSM303DLHC_ACC) || defined(LSM303D_ACC) || defined(MPU6050) || defined(LSM330) || defined(MMA8451Q) || defined(NUNCHUCK) || defined(LSM9DS0_ACC)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110) || defined(LSM9DS0_MAG)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(LSM330) || defined(MPU3050) || defined(WMP) || defined(L3GD20) || defined(LSM9DS0_GYRO)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(I2C_GPS_SONAR)
  #define SONAR 1
#else
  #define SONAR 0
#endif

#if !defined(ACC_1G)
  #define ACC_1G 256
#endif
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#if defined(L3GD20) || defined(LSM9DS0_GYRO)
  #define GYRO_SCALE ((70.0f * PI) / (180.0f * 1000000.0f * 1000.0f))
#endif


/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
  #define SERVO_RATES      {30,30,100,100,0,1,100,100}
#endif


#define RC_CHANS 8


/**************************************************************************************/
/***************               override defaults                   ********************/
/**************************************************************************************/
  #ifdef OVERRIDE_LEDPIN_PINMODE
    #undef LEDPIN_PINMODE
    #undef LEDPIN_TOGGLE
    #undef LEDPIN_OFF
    #undef LEDPIN_ON
    #define LEDPIN_PINMODE OVERRIDE_LEDPIN_PINMODE
    #define LEDPIN_TOGGLE  OVERRIDE_LEDPIN_TOGGLE
    #define LEDPIN_OFF     OVERRIDE_LEDPIN_OFF
    #define LEDPIN_ON      OVERRIDE_LEDPIN_ON
  #endif

  /*********  sensors orientation - possibly overriding board defaults  *****/
  #ifdef FORCE_GYRO_ORIENTATION
    #undef GYRO_ORIENTATION
    #define GYRO_ORIENTATION FORCE_GYRO_ORIENTATION
  #endif
  #ifdef FORCE_ACC_ORIENTATION
    #undef ACC_ORIENTATION
    #define ACC_ORIENTATION FORCE_ACC_ORIENTATION
  #endif
  #ifdef FORCE_MAG_ORIENTATION
    #undef MAG_ORIENTATION
    #define MAG_ORIENTATION FORCE_MAG_ORIENTATION
  #endif

#endif /* DEF_H_ */

