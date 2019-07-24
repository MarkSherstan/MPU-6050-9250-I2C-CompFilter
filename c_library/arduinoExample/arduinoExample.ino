// Include required librarys
#include <Wire.h>
#include "../MPU9250.h"

// Set up read and write functions
i2c_device_t i2c_dev;

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


void setup() {
  // Start
  Wire.begin();
  Serial.begin(115200);

  // Prepare I2C functions
  i2c_dev.i2c_write = i2c_write;
  i2c_dev.i2c_read = i2c_read;
  MPU9250 mpu9250(0x68, i2c_dev);

  // Initialize the IMU and set the senstivity values
  if (!mpu9250.initIMU()) Serial.println("IMU detection failed");
  mpu9250.getAres(AFS_4G);
  mpu9250.getGres(GFS_500DPS);

  // Calibrate the IMU
  if (!mpu9250.gyroCalibration(500)) Serial.println("Gyroscope calibration failed");
  if (!mpu9250.accelCalibration(AFS_4G)) Serial.println("Accelerometer calibration failed");
}

void loop() {
  // Read data
  mpu9250.readCalData();

  // Print raw data to the serial monitor
  Serial.print("Raw Data")
  Serial.print(mpu9250.imu_raw.ax,2); Serial.print(",");
  Serial.print(mpu9250.imu_raw.ay,2); Serial.print(",");
  Serial.print(mpu9250.imu_raw.az,2); Serial.print(",");
  Serial.print(mpu9250.imu_raw.gx,2); Serial.print(",");
  Serial.print(mpu9250.imu_raw.gy,2); Serial.print(",");
  Serial.println(mpu9250.imu_raw.gz,2);

  // Print calibrated data to the serial monitor
  Serial.print("Calibrated Data")
  Serial.print(mpu9250.imu_cal.ax,2); Serial.print(",");
  Serial.print(mpu9250.imu_cal.ay,2); Serial.print(",");
  Serial.print(mpu9250.imu_cal.az,2); Serial.print(",");
  Serial.print(mpu9250.imu_cal.gx,2); Serial.print(",");
  Serial.print(mpu9250.imu_cal.gy,2); Serial.print(",");
  Serial.println(mpu9250.imu_cal.gz,2);
}
