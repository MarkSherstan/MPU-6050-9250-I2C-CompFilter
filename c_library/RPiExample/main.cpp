#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include "mpuXX50.h"

// Some variable definitions
int fd;
float dt;
clock_t t;

// I2C read and write functions
int i2c_read(int addr, unsigned char *data, char len){
  char a = data[0];

  for (int ii = 0; ii < len; ii++){
    data[ii] = wiringPiI2CReadReg8(addr, a);
    a++;
  }

  return 0;
};

int i2c_write(int addr, unsigned char *data, char len){
  for (int ii = 1; ii < len; ii++){
    wiringPiI2CWriteReg8(addr, data[ii-1], data[ii]);
  }

  return 0;
};

// Setup the MPU class
struct i2c_device_t i2c_dev;
MPUXX50 *mpuXX50;

// Main function
int main(){
  // Start the wiringPiI2C on 0x68
  if ((fd = wiringPiI2CSetup(0x68)) < 0) {
    std::cout << "wiringPiI2CSetup failed" << std::endl;
    return -1;
  }

  // Prepare I2C functions for read and write
  i2c_dev.i2c_write = (i2c_read_write_t) &i2c_write;
  i2c_dev.i2c_read = (i2c_read_write_t) &i2c_read;

  // Connect to sensor using file identifier
  mpuXX50 = new MPUXX50(fd, i2c_dev);

  // Initialize the IMU and set the senstivity values
  std::cout << "IMU initialize. Pass/Fail: ";
  std::cout << mpuXX50->initIMU(MPU9250) << std::endl;
  mpuXX50->getAres(AFS_4G);
  mpuXX50->getGres(GFS_500DPS);
  sleep(1);

  // Calibrate the gyroscope
  gyro_cal_t gyro_cal;
  std::cout << "Calibrating gyroscope, hold IMU stationary." << std::endl;
  sleep(2);
  mpuXX50->gyroCalibration(1000);

  // Load saved gyroscope calibration values
  // gyro_cal.x = 0;
  // gyro_cal.y = 0;
  // gyro_cal.z = 0;
  // mpuXX50->setGyroCalibration(gyro_cal);

  // Display calibration values to user
  gyro_cal = mpuXX50->getGyroCalibration();

  std::cout << "---------------------------------------" << std::endl;
  std::cout << "Gyroscope bias values:" << std::endl;
  std::cout << "\t X: " << mpuXX50->gyro_cal.x << std::endl;
  std::cout << "\t Y: " << mpuXX50->gyro_cal.y << std::endl;
  std::cout << "\t Z: " << mpuXX50->gyro_cal.z << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  sleep(2);

  // Start a timer
  t = clock();

  // Run this forever
  while(true) {
    // Record the change in time
    t = clock() - t;
    dt = ((float)t) / CLOCKS_PER_SEC;
    std::cout << std::setprecision(3) << dt << "\t";

    // Print raw data
    // mpuXX50->readRawData();
    // std::cout << std::setprecision(3) << mpuXX50->imu_raw.ax << ", ";
    // std::cout << std::setprecision(3) << mpuXX50->imu_raw.ay << ", ";
    // std::cout << std::setprecision(3) << mpuXX50->imu_raw.az << " | ";
    // std::cout << std::setprecision(3) << mpuXX50->imu_raw.gx << ", ";
    // std::cout << std::setprecision(3) << mpuXX50->imu_raw.gy << ", ";
    // std::cout << std::setprecision(3) << mpuXX50->imu_raw.gz << std::endl;

    // Print calibrated data
    mpuXX50->readCalData();
    std::cout << std::setprecision(3) << mpuXX50->imu_cal.ax << ", ";
    std::cout << std::setprecision(3) << mpuXX50->imu_cal.ay << ", ";
    std::cout << std::setprecision(3) << mpuXX50->imu_cal.az << " | ";
    std::cout << std::setprecision(3) << mpuXX50->imu_cal.gx << ", ";
    std::cout << std::setprecision(3) << mpuXX50->imu_cal.gy << ", ";
    std::cout << std::setprecision(3) << mpuXX50->imu_cal.gz << std::endl;

    // Print complementary filter attitude
    // mpuXX50->compFilter(dt, 0.98);
    // std::cout << std::setprecision(1) << mpuXX50->attitude.roll << ", ";
    // std::cout << std::setprecision(1) << mpuXX50->attitude.pitch << ", ";
    // std::cout << std::setprecision(1) << mpuXX50->attitude.yaw << std::endl;
  }
}
