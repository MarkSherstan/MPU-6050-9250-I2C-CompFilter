#include "mpu9250.h"

MPU9250::MPU9250(char addr, i2c_device_t i2c_dev){
  _addr = addr;
  _i2c_dev = i2c_dev;
}

void MPU9250::initIMU() {
  // Check if a valid connection has been established
  char whoAmI = _i2c_dev.i2c_read(_addr, WHO_AM_I_MPU9250, 1);

  if (whoAmI == 0x71){
    // Activate/reset the IMU
    _i2c_dev.i2c_write(_addr, {PWR_MGMT_1, 0x00}, 2);

    return true;
    break;
  }

  return false;
}

float MPU9250::getAres(int Ascale) {
  // Set the full scale range for the accelerometer
  switch (Ascale){
    case AFS_2G:
      _aRes = 16384.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x00}, 2);
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 8192.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x08}, 2);
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 4096.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x10}, 2);
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 2048.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x18}, 2);
      return _aRes;
      break;
  }
}

float MPU9250::getGres(int Gscale) {
  // Set the full scale range for the gyroscope
  switch (Gscale){
    case GFS_250DPS:
      _gRes = 131.0;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x00}, 2);
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 65.5;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x08}, 2);
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 32.8;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x10}, 2);
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 16.4;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x18}, 2);
      return _gRes;
      break;
  }
}

void MPU9250::readRawData() {
  // Subroutine for reading the raw data
  _i2c_dev.i2c_write(_addr, ACCEL_XOUT_H, 1);
  _i2c_dev.i2c_read(_addr, ACCEL_XOUT_H, 14);

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
  mpu9250.readRawData();

  // Remove accelerometer offset and scale values
  imu_cal.ax = (imu_raw.ax - accel_cal.bx) / accel_cal.sx;
  imu_cal.ay = (imu_raw.ay - accel_cal.by) / accel_cal.sy;
  imu_cal.az = (imu_raw.az - accel_cal.bz) / accel_cal.sz;

  // Convert accelerometer values to g
  imu_cal.ax /= _aRes;
  imu_cal.ay /= _aRes;
  imu_cal.az /= _aRes;

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
  // Initialize standard deviation variabls
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
        break;
      }
    }
  }

  // Average the averge offset values and return true if everything passes
  gyro_cal.x /= numCalPoints;
  gyro_cal.y /= numCalPoints;
  gyro_cal.z /= numCalPoints;

  return true;
}

bool MPU9250::accelCalibration(int numCalPointsPerAxis) {
  // Initialize arrays for calibration
  int max[3] = {-32767, -32767, -32767}, min[3] = {32767, 32767, 32767}, temp[3] = {0, 0, 0};

  // Loop through each axis (X, y, z) recording max and min values
  for (ii = 0; ii < 3; ii++){
    for(jj = 0; jj < numCalPointsPerAxis; jj++){
      mpu9250.readRawData();

      for (int kk = 0; kk < 3; kk++){
        if(temp[kk] > max[kk]) max[kk] = temp[kk];
        if(temp[kk] < min[kk]) min[kk] = temp[kk];
      }
    }
    // Delay for user to swtich axis
  }

  // Do scaling or bias removal calculations here.
}

gyro_calib_t MPU9250::getGyroCalibration() {
  return gyro_cal;
}

accel_calib_t MPU9250::getAccelCalibration() {
  return accel_cal;
}
