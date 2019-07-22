#include "mpu9250.h"

MPU9250::MPU9250(char addr, i2c_device_t i2c_dev){
  _addr = addr;
  _i2c_dev = i2c_dev;
}

void MPU9250::readRawData() {
  // Subroutine for reading the raw data
  data[0] = 0x3B;
  _i2c_dev.i2c_write(_addr, data, 1);
  _i2c_dev.i2c_read(_addr, data, 14);

  // Read data --> Temperature falls between acc and gyro registers
  imu_raw.ax = data[0]  << 8 | data[1];
  imu_raw.ay = data[2]  << 8 | data[3];
  imu_raw.az = data[4]  << 8 | data[5];

  imu_raw.gx = data[8]  << 8 | data[9];
  imu_raw.gy = data[10] << 8 | data[11];
  imu_raw.gz = data[12] << 8 | data[13];

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

bool MPU9250::gyroCalibration(int numCalPoints) {
  // Initialize arrays for calibration checks
  float std2_x, std2_y, std2_z;
  float numeratorX, numeratorY, numeratorZ;

  // Run calibration for given number of points
  for (int ii = 0; ii < numCalPoints; ii++){
    mpu9250.readRawData();
    gyro_cal.x += imu_raw.gx;
    gyro_cal.y += imu_raw.gy;
    gyro_cal.z += imu_raw.gz;

    // Standard deviation numerator calculation
    numeratorX += (imu_raw.gx - (gyro_cal.x / (ii+1)) )^2;
    numeratorY += (imu_raw.gy - (gyro_cal.y / (ii+1)) )^2;
    numeratorZ += (imu_raw.gz - (gyro_cal.z / (ii+1)) )^2;

    // Build a small sample before checking if gyro values are within 2 standard deviations
    if (ii > 5){
      std2_x = sqrt(numeratorX / ii) * 2;
      std2_y = sqrt(numeratorY / ii) * 2;
      std2_z = sqrt(numeratorZ / ii) * 2;

      if ((abs(imu_raw.gx) > std2_x) || (abs(imu_raw.gy) > std2_y) || (abs(imu_raw.gz) > std2_z) ){
        return false;
      }
    }
  }

  // Average the averge offset values and return true if everything passes
  gyro_cal.x /= numCalPoints;
  gyro_cal.y /= numCalPoints;
  gyro_cal.z /= numCalPoints;

  return true;
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
