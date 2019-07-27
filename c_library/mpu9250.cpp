#include "mpu9250.h"

#ifdef ARDUINO
#include <arduino.h>
#else
#include <math.h>
#endif

MPU9250::MPU9250(char addr, i2c_device_t i2c_dev){
  _addr = addr;
  _i2c_dev = i2c_dev;
}

bool MPU9250::initIMU() {
  // Check if a valid connection has been established
  data[0] = WHO_AM_I_MPU9250;
  _i2c_dev.i2c_write(_addr, data, 1);
  _i2c_dev.i2c_read(_addr, data, 1);
  char whoAmI = data[0];

  if (whoAmI == 0x71 || whoAmI == 0x68){
    // Activate/reset the IMU
    write2bytes(PWR_MGMT_1, 0x00);
    return true;
  }

  return false;
}

int MPU9250::write2bytes(char byte0, char byte1) {
  data[0] = byte0;
  data[1] = byte1;
  _i2c_dev.i2c_write(_addr, data, 2);
}

float MPU9250::getAres(int Ascale) {
  // Set the full scale range for the accelerometer
  switch (Ascale){
    case AFS_2G:
      _aRes = 16384.0;
      write2bytes(ACCEL_CONFIG, 0x00);
      return _aRes;
    case AFS_4G:
      _aRes = 8192.0;
      write2bytes(ACCEL_CONFIG, 0x08);
      return _aRes;
    case AFS_8G:
      _aRes = 4096.0;
      write2bytes(ACCEL_CONFIG, 0x10);
      return _aRes;
    case AFS_16G:
      _aRes = 2048.0;
      write2bytes(ACCEL_CONFIG, 0x18);
      return _aRes;
    default:
      return 0;
  }
}

float MPU9250::getGres(int Gscale) {
  // Set the full scale range for the gyroscope
  switch (Gscale){
    case GFS_250DPS:
      _gRes = 131.0;
      write2bytes(GYRO_CONFIG, 0x00);
      return _gRes;
    case GFS_500DPS:
      _gRes = 65.5;
      write2bytes(GYRO_CONFIG, 0x08);
      return _gRes;
    case GFS_1000DPS:
      _gRes = 32.8;
      write2bytes(GYRO_CONFIG, 0x10);
      return _gRes;
    case GFS_2000DPS:
      _gRes = 16.4;
      write2bytes(GYRO_CONFIG, 0x18);
      return _gRes;
    default:
      return 0;
  }
}

void MPU9250::readRawData() {
  // Subroutine for reading the raw data
  data[0] = ACCEL_XOUT_H;
  _i2c_dev.i2c_write(_addr, data, 1);
  _i2c_dev.i2c_read(_addr, data, 14);

  // Read data - Temperature falls between accel and gyro registers
  imu_raw.ax = data[0]  << 8 | data[1];
  imu_raw.ay = data[2]  << 8 | data[3];
  imu_raw.az = data[4]  << 8 | data[5];

  imu_raw.gx = data[8]  << 8 | data[9];
  imu_raw.gy = data[10] << 8 | data[11];
  imu_raw.gz = data[12] << 8 | data[13];

  temperature = data[6] << 8 | data[7];
}

void MPU9250::readCalData() {
  // Get new data
  readRawData();

  // Convert accelerometer values to g's
  imu_cal.ax = imu_raw.ax / _aRes;
  imu_cal.ay = imu_raw.ay / _aRes;
  imu_cal.az = imu_raw.az / _aRes;

  // Remove gyro offset
  imu_cal.gx = imu_raw.gx - gyro_cal.x;
  imu_cal.gy = imu_raw.gy - gyro_cal.y;
  imu_cal.gz = imu_raw.gz - gyro_cal.z;

  // Convert gyro values to degrees per second
  imu_cal.gx /= _gRes;
  imu_cal.gy /= _gRes;
  imu_cal.gz /= _gRes;
}

bool MPU9250::gyroCalibration(int numCalPoints) {
  // Initialize standard deviation and check variabls
  float stdX, stdY, stdZ;
  float xCheckL, yCheckL, zCheckL;
  float xCheckH, yCheckH, zCheckH;
  float numeratorX, numeratorY, numeratorZ;

  // Run calibration for given number of points
  for (int ii = 0; ii < numCalPoints; ii++){
    readRawData();
    gyro_cal.x += imu_raw.gx;
    gyro_cal.y += imu_raw.gy;
    gyro_cal.z += imu_raw.gz;

    // Standard deviation numerator calculation
    numeratorX += ((imu_raw.gx - (gyro_cal.x / (ii+1)) ) * (imu_raw.gx - (gyro_cal.x / (ii+1)) ));
    numeratorY += ((imu_raw.gy - (gyro_cal.y / (ii+1)) ) * (imu_raw.gy - (gyro_cal.y / (ii+1)) ));
    numeratorZ += ((imu_raw.gz - (gyro_cal.z / (ii+1)) ) * (imu_raw.gz - (gyro_cal.z / (ii+1)) ));

    // Build a small sample before checking if current gyro values are within a standard deviation
    if (ii > 25){
      stdX = sqrt(numeratorX / (ii+1));
      stdY = sqrt(numeratorY / (ii+1));
      stdZ = sqrt(numeratorZ / (ii+1));

      xCheckH = (gyro_cal.x / (ii+1)) + stdX;
      yCheckH = (gyro_cal.y / (ii+1)) + stdY;
      zCheckH = (gyro_cal.z / (ii+1)) + stdZ;

      xCheckL = (gyro_cal.x / (ii+1)) - stdX;
      yCheckL = (gyro_cal.y / (ii+1)) - stdY;
      zCheckL = (gyro_cal.z / (ii+1)) - stdZ;

      if ((imu_raw.gx >= xCheckL && imu_raw.gx <= xCheckH)
            || (imu_raw.gy >= yCheckL && imu_raw.gy <= yCheckH)
                || (imu_raw.gz >= zCheckL && imu_raw.gz <= zCheckH) ){
        continue;
      } else {
        return false;
        }
      }
    }

  // Find the averge offset values and return true if everything passes
  gyro_cal.x /= (float)numCalPoints;
  gyro_cal.y /= (float)numCalPoints;
  gyro_cal.z /= (float)numCalPoints;

  return true;
}

void MPU9250::setGyroCalibration(gyro_cal_t gyro) {
  gyro_cal = gyro;
}

gyro_cal_t MPU9250::getGyroCalibration() {
  return gyro_cal;
}
