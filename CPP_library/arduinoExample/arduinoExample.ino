// Include librarys
#include <Wire.h>
#include "MPUXX50.h"

// Variable definition
unsigned long loopTimer;
float dt;

// Setup class
struct i2c_device_t i2c_dev;
MPUXX50 *mpuXX50;

// Sensor setup and calibration
void setup() {
  // Start wire and serial
  Wire.begin();
  Serial.begin(115200);

  // Prepare I2C functions
  i2c_dev.i2c_write = (i2c_read_write_t) &i2c_write;
  i2c_dev.i2c_read = (i2c_read_write_t) &i2c_read;

  // Connect to sensor on 0x68
  mpuXX50 = new MPUXX50(0x68, i2c_dev);

  // Initialize the IMU and set the senstivity values
  Serial.println("---------------------------------------");
  Serial.print("IMU initialize. Pass/Fail: "); Serial.println(mpuXX50->initIMU(MPU6050));
  mpuXX50->getAres(AFS_4G);
  mpuXX50->getGres(GFS_500DPS);
  delay(1000);

  // Calibrate the gyroscope
  gyro_cal_t gyro_cal;
  Serial.println("Calibrating gyroscope, hold IMU stationary."); delay(2000);
  mpuXX50->gyroCalibration(1000);

  // Load saved gyroscope calibration values
  // gyro_cal.x = 0;
  // gyro_cal.y = 0;
  // gyro_cal.z = 0;
  // mpuXX50->setGyroCalibration(gyro_cal);

  // Display calibration values to user
  gyro_cal = mpuXX50->getGyroCalibration();

  Serial.println("---------------------------------------");
  Serial.println("Gyroscope bias values:");
  Serial.print(mpuXX50->gyro_cal.x); Serial.print(",");
  Serial.print(mpuXX50->gyro_cal.y); Serial.print(",");
  Serial.println(mpuXX50->gyro_cal.z);
  Serial.println("---------------------------------------");
  delay(2000);

  // Start a timer
  loopTimer = micros();
}

// Sensor processing and printing
void loop() {
  // Loop timing (4000 us = 250 Hz)
  while (micros() - loopTimer <= 4000);
  dt = (float)(micros() - loopTimer) / 1000000;
  loopTimer = micros();

  // Print raw data to the serial monitor
  // mpuXX50->readRawData();
  //
  // Serial.print(mpuXX50->imu_raw.ax,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_raw.ay,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_raw.az,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_raw.gx,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_raw.gy,2); Serial.print(" , ");
  // Serial.println(mpuXX50->imu_raw.gz,2);

  // Print calibrated data to the serial monitor
  // mpuXX50->readCalData();
  //
  // Serial.print(mpuXX50->imu_cal.ax,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_cal.ay,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_cal.az,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_cal.gx,2); Serial.print(" , ");
  // Serial.print(mpuXX50->imu_cal.gy,2); Serial.print(" , ");
  // Serial.println(mpuXX50->imu_cal.gz,2);

  // Print complementary filter attitude to the serial monitor
  mpuXX50->compFilter(dt, 0.98);

  Serial.print(mpuXX50->attitude.roll,2); Serial.print(" , ");
  Serial.print(mpuXX50->attitude.pitch,2); Serial.print(" , ");
  Serial.println(mpuXX50->attitude.yaw,2);
}

// I2C read and write functions
int i2c_read(char addr, byte *data, char len){
  Wire.requestFrom(addr, len);

  for (int ii = 0; ii < len; ii++){
    data[ii] = Wire.read();
  }
};

int i2c_write(char addr, byte *data, char len){
  Wire.beginTransmission(addr);

  for (int ii = 0; ii < len; ii++){
    Wire.write(data[ii]);
  }

  Wire.endTransmission();
};
