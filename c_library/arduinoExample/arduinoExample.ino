// Include librarys
#include <Wire.h>
#include "MPU9250.h"

// Read function
int i2c_read(char addr, char *data, char len){
  Wire.requestFrom(addr, len);

  for (int ii = 0; ii < len; ii++){
    data[ii] = Wire.read();
  }
};

// Write function
int i2c_write(char addr, char *data, char len){
  Wire.beginTransmission(addr);

  for (int ii = 0; ii < len; ii++){
    Wire.write(data[ii]);
  }

  Wire.endTransmission();
};

// Setup class
struct i2c_device_t i2c_dev;
MPU9250 *mpu9250;


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
  if (!mpu9250->initIMU()) Serial.println("IMU detection failed");
  mpu9250->getAres(AFS_4G);
  mpu9250->getGres(GFS_500DPS);

  // Calibrate the IMU
  if (!mpu9250->gyroCalibration(500)) Serial.println("Gyroscope calibration failed");
  if (!mpu9250->accelCalibration(AFS_4G)) Serial.println("Accelerometer calibration failed");

  // Get the calibration values
  gyro_cal_t gyro_cal = mpu9250->getGyroCalibration();
  accel_cal_t accel_cal = mpu9250->getAccelCalibration();

  // Load saved calibration values
  // gyro_cal_t gyro_cal;
  // gyro_cal.x = 0;
  // gyro_cal.y = 0;
  // gyro_cal.z = 0;
  // mpu9250->setGyroCalibration(gyro_cal);
  //
  // accel_cal_t accel_cal;
  // accel_cal.sx = 1; accel_cal.bx = 0;
  // accel_cal.sy = 1; accel_cal.by = 0;
  // accel_cal.sz = 1; accel_cal.bz = 0;
  // mpu9250->setAccelCalibration(accel_cal);
}



void loop() {
  // Read data
  mpu9250->readCalData();

  // Print raw data to the serial monitor
  Serial.print("Raw Data");
  Serial.print(mpu9250->imu_raw.ax,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.ay,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.az,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.gx,2); Serial.print(",");
  Serial.print(mpu9250->imu_raw.gy,2); Serial.print(",");
  Serial.println(mpu9250->imu_raw.gz,2);

  // Print calibrated data to the serial monitor
  Serial.print("Calibrated Data");
  Serial.print(mpu9250->imu_cal.ax,2); Serial.print(",");
  Serial.print(mpu9250->imu_cal.ay,2); Serial.print(",");
  Serial.print(mpu9250->imu_cal.az,2); Serial.print(",");
  Serial.print(mpu9250->imu_cal.gx,2); Serial.print(",");
  Serial.print(mpu9250->imu_cal.gy,2); Serial.print(",");
  Serial.println(mpu9250->imu_cal.gz,2);
}
