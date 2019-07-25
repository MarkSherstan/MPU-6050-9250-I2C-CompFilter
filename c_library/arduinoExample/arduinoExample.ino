// Include librarys
#include <Wire.h>
#include "MPU9250.h"

// Variable definition
long loopTimer;

// Read function
int i2c_read(char addr, byte *data, char len){
  Wire.requestFrom(addr, len);

  for (int ii = 0; ii < len; ii++){
    data[ii] = Wire.read();
  }
};

// Write function
int i2c_write(char addr, byte *data, char len){
  Wire.beginTransmission(addr);

  for (int ii = 0; ii < len; ii++){
    Wire.write(data[ii]);
  }

  Wire.endTransmission();
};

// Setup class
struct i2c_device_t i2c_dev;
MPU9250 *mpu9250;

// Setup
void setup() {
  // Start wire and serial
  Wire.begin();
  Serial.begin(115200);

  // Prepare I2C functions
  i2c_dev.i2c_write = (i2c_read_write_t) &i2c_write;
  i2c_dev.i2c_read = (i2c_read_write_t) &i2c_read;

  // Connect to sensor on 0x68
  mpu9250 = new MPU9250(0x68, i2c_dev);

  // Initialize the IMU and set the senstivity values
  Serial.println("---------------------------------------");
  Serial.print("IMU initialized: "); Serial.println(mpu9250->initIMU());
  mpu9250->getAres(AFS_4G);
  mpu9250->getGres(GFS_500DPS);

  // Calibrate the IMU
  // Serial.println("Calibrating gyroscope, hold IMU stationary"); delay(2000);
  // if (!mpu9250->gyroCalibration(500)) Serial.println("Gyroscope calibration failed");
  //
  // Serial.println("Calibrating accelerometer, hold IMU stationary in all six directions"); delay(2000);
  // if (!mpu9250->accelCalibration(AFS_4G)) Serial.println("Accelerometer calibration failed");

  // Load saved calibration values
  gyro_cal_t gyro_cal;
  gyro_cal.x = 0;
  gyro_cal.y = 0;
  gyro_cal.z = 0;
  mpu9250->setGyroCalibration(gyro_cal);

  accel_cal_t accel_cal;
  accel_cal.sx = 1; accel_cal.bx = 0;
  accel_cal.sy = 1; accel_cal.by = 0;
  accel_cal.sz = 1; accel_cal.bz = 0;
  mpu9250->setAccelCalibration(accel_cal);

  // Get the calibration values and display for user
  Serial.println("---------------------------------------");

  gyro_cal = mpu9250->getGyroCalibration();
  Serial.println("Gyroscope bias values:");
  Serial.print(mpu9250->gyro_cal.x); Serial.print(",");
  Serial.print(mpu9250->gyro_cal.y); Serial.print(",");
  Serial.println(mpu9250->gyro_cal.z);

  accel_cal = mpu9250->getAccelCalibration();
  Serial.println("\nAccelerometer bias values:");
  Serial.print(mpu9250->accel_cal.bx); Serial.print(",");
  Serial.print(mpu9250->accel_cal.by); Serial.print(",");
  Serial.println(mpu9250->accel_cal.bz);

  Serial.println("\nAccelerometer scale values:");
  Serial.print(mpu9250->accel_cal.sx); Serial.print(",");
  Serial.print(mpu9250->accel_cal.sy); Serial.print(",");
  Serial.println(mpu9250->accel_cal.sz); Serial.println();

  Serial.println("---------------------------------------");
  delay(2000);

  // Start a timer
  loopTimer = micros();
}


void loop() {
  // Read data
  mpu9250->readRawData();

  // Print raw data to the serial monitor
  Serial.print(mpu9250->imu_raw.ax,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.ay,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.az,2); Serial.print("\t");
  Serial.print(mpu9250->imu_raw.gx,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.gy,2); Serial.print(",");
  Serial.println(mpu9250->imu_raw.gz,2);

  // Print calibrated data to the serial monitor
  // Serial.print(mpu9250->imu_cal.ax,2); Serial.print(",");
  // Serial.print(mpu9250->imu_cal.ay,2); Serial.print(",");
  // Serial.print(mpu9250->imu_cal.az,2); Serial.print("\t");
  // Serial.print(mpu9250->imu_cal.gx,2); Serial.print(",");
  // Serial.print(mpu9250->imu_cal.gy,2); Serial.print(",");
  // Serial.println(mpu9250->imu_cal.gz,2);

  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000);
  loopTimer = micros();
}
