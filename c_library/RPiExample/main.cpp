// sudo apt-get install libi2c-dev

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include "mpuXX50.h"

using namespace std;

// Some variable definitions
int fd;
float dt;
clock_t t;

// I2C read and write functions
int i2c_read(int addr, unsigned char *data, char len){
  char a = data[0];

  for (int ii = 0; ii < len; ii++){
    // cout << hex << a << "\t";
    data[ii] = wiringPiI2CReadReg8(addr, a);
    a++;
  }

  cout << endl;

  return 0;
};

int i2c_write(int addr, unsigned char *data, char len){
  for (int ii = 1; ii < len; ii++){
    // wiringPiI2CWrite(addr, data[ii]);
    wiringPiI2CWriteReg8(addr, data[ii-1], data[ii]);
  }

  return 0;
};

// Setup class
struct i2c_device_t i2c_dev;
MPUXX50 *mpuXX50;

// Main function
int main(){

  // Start the wiringPiI2C on 0x68
  if ((fd = wiringPiI2CSetup(0x68)) < 0) {
    cout << "wiringPiI2CSetup failed" << endl;
    return -1;
  }

  // Prepare I2C functions
  i2c_dev.i2c_write = (i2c_read_write_t) &i2c_write;
  i2c_dev.i2c_read = (i2c_read_write_t) &i2c_read;

  // Connect to sensor using file identifier
  mpuXX50 = new MPUXX50(fd, i2c_dev);

  // Initialize the IMU and set the senstivity values
  cout << "IMU initialize. Pass/Fail: ";
  cout << mpuXX50->initIMU(MPU9250) << endl;
  mpuXX50->getAres(AFS_4G);
  mpuXX50->getGres(GFS_500DPS);
  sleep(1);

  // Calibrate the gyroscope
  gyro_cal_t gyro_cal;
  cout << "Calibrating gyroscope, hold IMU stationary." << endl;
  sleep(2);
  mpuXX50->gyroCalibration(1000);

  // Load saved gyroscope calibration values
  // gyro_cal.x = 0;
  // gyro_cal.y = 0;
  // gyro_cal.z = 0;
  // mpuXX50->setGyroCalibration(gyro_cal);

  // Display calibration values to user
  gyro_cal = mpuXX50->getGyroCalibration();

  cout << "---------------------------------------" << endl;
  cout << "Gyroscope bias values:" << endl;
  cout << mpuXX50->gyro_cal.x << "," << mpuXX50->gyro_cal.y << "," << mpuXX50->gyro_cal.z << endl;
  cout << "---------------------------------------" << endl;
  sleep(2);

  // Start a timer
  t = clock();


  while(true) {

    t = clock() - t;
    dt = ((float)t) / CLOCKS_PER_SEC;

    cout << dt << "\t";

    // // Print raw data
    // mpuXX50->readRawData();
    // cout << mpuXX50->imu_raw.ax << " , ";
    // cout << mpuXX50->imu_raw.ay << " , ";
    // cout << mpuXX50->imu_raw.az << " , ";
    // cout << mpuXX50->imu_raw.gx << " , ";
    // cout << mpuXX50->imu_raw.gy << " , ";
    // cout << mpuXX50->imu_raw.gz << endl;

    // Print calibrated data
    mpuXX50->readCalData();
    cout << mpuXX50->imu_cal.ax << " , ";
    cout << mpuXX50->imu_cal.ay << " , ";
    cout << mpuXX50->imu_cal.az << " , ";
    cout << mpuXX50->imu_cal.gx << " , ";
    cout << mpuXX50->imu_cal.gy << " , ";
    cout << mpuXX50->imu_cal.gz << endl;


    // Print complementary filter attitude
    // mpuXX50->compFilter(dt, 0.98);
    //
    // cout << mpuXX50->attitude.roll << " , ";
    // cout << mpuXX50->attitude.pitch << " , ";
    // cout << mpuXX50->attitude.yaw << endl;


    usleep(500000);
  }

}
