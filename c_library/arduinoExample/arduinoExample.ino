// Include librarys
#include <Wire.h>
#include "MPU9250.h"

// Variable definition
long loopTimer;

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
  Serial.println("---------------------------------------");
  Serial.print("IMU initialize. Pass/Fail: "); Serial.println(mpu9250->initIMU());
  mpu9250->getAres(AFS_4G);
  mpu9250->getGres(GFS_500DPS);

  // Flush out initial data
  for (int cal_int = 0; cal_int < 5; cal_int ++){
    mpu9250->readRawData();
    delay(100);
  }

  // Calibrate the gyroscope
  gyro_cal_t gyro_cal;
  Serial.print("Calibrating gyroscope, hold IMU stationary. Pass/Fail: "); delay(2000);
  Serial.println(mpu9250->gyroCalibration(1000));

  // Load saved gyroscope calibration values
  // gyro_cal.x = 0;
  // gyro_cal.y = 0;
  // gyro_cal.z = 0;
  // mpu9250->setGyroCalibration(gyro_cal);

  // Display calibration values to user
  gyro_cal = mpu9250->getGyroCalibration();

  Serial.println("---------------------------------------");
  Serial.println("Gyroscope bias values:");
  Serial.print(mpu9250->gyro_cal.x); Serial.print(",");
  Serial.print(mpu9250->gyro_cal.y); Serial.print(",");
  Serial.println(mpu9250->gyro_cal.z);
  Serial.println("---------------------------------------");
  delay(2000);

  // Start a timer
  loopTimer = micros();
}



void loop() {
  // Read data
  mpu9250->readCalData();

  // Print raw data to the serial monitor
  // Serial.print(mpu9250->imu_raw.ax,2); Serial.print(" , ");
  // Serial.print(mpu9250->imu_raw.ay,2); Serial.print(" , ");
  // Serial.print(mpu9250->imu_raw.az,2); Serial.print(" , ");
  // Serial.print(mpu9250->imu_raw.gx,2); Serial.print(" , ");
  // Serial.print(mpu9250->imu_raw.gy,2); Serial.print(" , ");
  // Serial.println(mpu9250->imu_raw.gz,2);

  // Print calibrated data to the serial monitor
  Serial.print(mpu9250->imu_cal.ax,2); Serial.print(" , ");
  Serial.print(mpu9250->imu_cal.ay,2); Serial.print(" , ");
  Serial.print(mpu9250->imu_cal.az,2); Serial.print(" , ");
  Serial.print(mpu9250->imu_cal.gx,2); Serial.print(" , ");
  Serial.print(mpu9250->imu_cal.gy,2); Serial.print(" , ");
  Serial.println(mpu9250->imu_cal.gz,2);

  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000);
  loopTimer = micros();
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
