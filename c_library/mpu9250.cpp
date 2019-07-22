#include "mpu9250.h"

MPU9250::MPU9250(char addr, i2c_device_t i2c_dev){
  _addr = addr;
  _i2c_dev = i2c_dev;
}

void MPU9250::readData() {
  // Subroutine for reading the raw data
  data[0] = 0x3B;
  _i2c_dev.i2c_write(_addr, data, 1);
  _i2c_dev.i2c_read(_addr, data, 14);

  // Read data --> Temperature falls between acc and gyro registers
  imu.ax = data[0]   << 8 | data[1];
  imu.ay = data[2]   << 8 | data[3];
  imu.az = data[4]   << 8 | data[5];

  imu.gx = data[8]  << 8 | data[9];
  imu.gy = data[10] << 8 | data[11];
  imu.gz = data[12] << 8 | data[13];

  temp   = data[6]  << 8 | data[7];
}

void MPU9250::setUpRegisters() {
  //Activate the MPU-6050
  data[0] = 0x6B;
  data[1] = 0x00;
  _i2c_dev.i2c_write(_addr, data, 2);

  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  data[0] = 0x1C;
  data[1] = 0x08;
  _i2c_dev.i2c_write(_addr, data, 2);

  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  data[0] = 0x1B;
  data[1] = 0x08;
  _i2c_dev.i2c_write(_addr, data, 2);
}

void MPU9250::startGyroCalibration() {

}

gyro_calib_t MPU9250::getGyroCalibration() {
  return gyro_cal;
}

accel_calib_t MPU9250::getAccelCalibration() {
  return accel_cal;
}

mag_calib_t MPU9250::getMagCalibration() {
  return mag_cal;
}
