// Include gaurd
#ifndef MPU9250_H
#define MPU9250_H

// Type def for function pointer
typedef int (*i2c_read_write_t)(char addr, char *data, char len);

// Read and write
struct i2c_device_t {
  i2c_read_write_t i2c_write;
  i2c_read_write_t i2c_read;
};

// IMU data structure
struct imu_t {
  float ax, ay, az, gx, gy, gz;
};

//
struct gyro_calib_t {
  float gXcal, gYcal, gZcal;
  // Standard deviation thing
};

struct accel_calib_t {
  float aXcal, aYcal, aZcal;
  // Normalize later
};

struct mag_calib_t {
  float magXbias, magYbias, magZbias;
  float magXscale, magYscale, magZscale;
};


class MPU9250 {
private:
  char _addr;
  i2c_device_t _i2c_dev;

  char data[14];

public:
  MPU9250(char addr, i2c_device_t i2c_dev);

  bool startGyroCalibration();
  bool startAccelCalibration();
  bool startMagCalibration();

  gyro_calib_t getGyroCalibration();
  accel_calib_t getAccelCalibration();
  mag_calib_t getMagCalibration();

  void readData();
  void readMagData();
  void setUpRegisters();

  imu_t imu_raw;
  imu_t imu_cal;

  gyro_calib_t gyro_cal;
  accel_calib_t accel_cal;
  mag_calib_t mag_cal;

  int temp;
}

#endif //MPU9250_H
