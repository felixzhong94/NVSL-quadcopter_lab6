#ifndef SENSORS_H_
#define SENSORS_H_

#if defined(MMA7455) || defined(MMA8451Q) || defined(ADXL345) || \
    defined(BMA180) || defined(BMA280) || defined(BMA020) || defined(NUNCHACK) || \
    defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(ADCACC) || \
    defined(MPU6050) || defined(LSM330) || defined(NUNCHUCK) || defined(LSM303DLHC_ACC) || defined(LSM303D_ACC) || defined(LSM9DS0_ACC)
void ACC_getADC ();
#endif

#if defined(L3G4200D) || defined(ITG3200) || defined(MPU6050) || defined(LSM330) || \
    defined(MPU3050) || defined(WMP) || defined(NUNCHUCK) || defined(L3GD20) || defined(LSM9DS0_GYRO)
void Gyro_getADC ();
#endif

#if MAG
uint8_t Mag_getADC();
#endif


void initSensors();
void i2c_rep_start(uint8_t address);
void i2c_write(uint8_t data );
void i2c_stop(void);
void i2c_write(uint8_t data );
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
uint8_t i2c_readAck();
uint8_t i2c_readNak();


#endif /* SENSORS_H_ */
