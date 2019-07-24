#include "mpu9250.h"

MPU9250::MPU9250(char addr, i2c_device_t i2c_dev){
  _addr = addr;
  _i2c_dev = i2c_dev;
}

void MPU9250::initIMU() {
  // Check if a valid connection has been established
  data[0] = WHO_AM_I_MPU9250;
  _i2c_dev.i2c_write(_addr, data, 1);
  _i2c_dev.i2c_read(_addr, data, 1);
  whoAmI = data[0];

  if (whoAmI == 0x71){
    // Activate/reset the IMU
    _i2c_dev.i2c_write(_addr, {PWR_MGMT_1, 0x00}, 2);

    return true;
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
    case AFS_4G:
      _aRes = 8192.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x08}, 2);
      return _aRes;
    case AFS_8G:
      _aRes = 4096.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x10}, 2);
      return _aRes;
    case AFS_16G:
      _aRes = 2048.0;
      _i2c_dev.i2c_write(_addr, {ACCEL_CONFIG, 0x18}, 2);
      return _aRes;
    case default:
      return 0;
  }
}

float MPU9250::getGres(int Gscale) {
  // Set the full scale range for the gyroscope
  switch (Gscale){
    case GFS_250DPS:
      _gRes = 131.0;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x00}, 2);
      return _gRes;
    case GFS_500DPS:
      _gRes = 65.5;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x08}, 2);
      return _gRes;
    case GFS_1000DPS:
      _gRes = 32.8;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x10}, 2);
      return _gRes;
    case GFS_2000DPS:
      _gRes = 16.4;
      _i2c_dev.i2c_write(_addr, {GYRO_CONFIG, 0x18}, 2);
      return _gRes;
    case default:
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
      }
    }
  }

  // Average the averge offset values and return true if everything passes
  gyro_cal.x /= numCalPoints;
  gyro_cal.y /= numCalPoints;
  gyro_cal.z /= numCalPoints;

  return true;
}

bool MPU9250::accelCalibration(int Ascale) {
  // Initialize arrays for calibration
  int max[3] = {-32767, -32767, -32767}, min[3] = {32767, 32767, 32767}, temp[3] = {0, 0, 0};
  float numeratorX, numeratorY, numeratorZ;
  float stdX, stdY, stdZ;
  float sumX, sumY, sumZ;

  // Record max and min values for each axis. Maximum of 100 attempts
  for (int ii = 0; ii < 100; ii++){

    // Build a small sample before running checks
    for(int jj = 1; jj < 6; jj++){
      // Read new data
      mpu9250.readRawData();

      // Sum values for calculating average (x bar)
      sumX += imu_raw.ax;
      sumY += imu_raw.ay;
      sumZ += imu_raw.az;

      // Standard deviation numerator calculation
      numeratorX += (imu_raw.ax - (sumX / jj) )^2;
      numeratorY += (imu_raw.ay - (sumY / jj) )^2;
      numeratorZ += (imu_raw.az - (sumZ / jj) )^2;
    }

    // Run the checks and record data
    while(true){
      // Read new data and increase counter
      mpu9250.readRawData();
      jj += 1;

      // Sum values for calculating average (x bar)
      sumX += imu_raw.ax;
      sumY += imu_raw.ay;
      sumZ += imu_raw.az;

      // Standard deviation numerator calculation
      numeratorX += (imu_raw.ax - (sumX / jj) )^2;
      numeratorY += (imu_raw.ay - (sumY / jj) )^2;
      numeratorZ += (imu_raw.az - (sumZ / jj) )^2;

      // Calculate standard deviation with a scaling factor
      stdX = sqrt(numeratorX / jj) * 0.2;
      stdY = sqrt(numeratorY / jj) * 0.2;
      stdZ = sqrt(numeratorZ / jj) * 0.2;

      // If accel is stationary record data otherwise reset
      if ((abs(imu_raw.ax) > stdX) || (abs(imu_raw.ay) > stdY) || (abs(imu_raw.az) > stdZ) ){
        // Stop recording data and reset values
        numeratorX = 0; numeratorY = 0; numeratorZ = 0;
        sumX = 0; sumY = 0; sumZ = 0;
        break
      } else {
        // Store the largest and smallest value
        temp[0] = imu_raw.ax;
        temp[1] = imu_raw.ay;
        temp[2] = imu_raw.az;

        for (int kk = 0; kk < 3; kk++){
          if(temp[kk] > max[kk]) max[kk] = temp[kk];
          if(temp[kk] < min[kk]) min[kk] = temp[kk];
        }
      }
    }
  }

  if (max[0] < 0 || max[1] < 0 || max[2] < 0 || min[0] > 0 || min[1] > 0 || min[2] > 0){
    // Failed calibration
    return false;
  } else {
  // Bias calculation
  accel_cal.bx = (max[0] + min[0]) / 2;
  accel_cal.by = (max[1] + min[1]) / 2;
  accel_cal.bz = (max[2] + min[2]) / 2;

  // Scaling calculation
  accel_cal.sx = (max[0] - min[0]) / (2 * _aRes);
  accel_cal.sy = (max[1] - min[1]) / (2 * _aRes);
  accel_cal.sz = (max[2] - min[2]) / (2 * _aRes);

  return true;
  }
}

gyro_calib_t MPU9250::getGyroCalibration() {
  return gyro_cal;
}

accel_calib_t MPU9250::getAccelCalibration() {
  return accel_cal;
}
