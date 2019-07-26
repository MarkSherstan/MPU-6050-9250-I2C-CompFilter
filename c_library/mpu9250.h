// Include gaurd
#ifndef MPU9250_H
#define MPU9250_H

// IMU configuration and check registries
#define WHO_AM_I_MPU9250  0x75
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C

// Other IMU registeries
#define PWR_MGMT_1  0x6B

// Accelerometer, temperature, and gyroscope data out registries
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42

#define GYRO_XOUT_H   0x43
#define GYRO_XOUT_L   0x44
#define GYRO_YOUT_H   0x45
#define GYRO_YOUT_L   0x46
#define GYRO_ZOUT_H   0x47
#define GYRO_ZOUT_L   0x48

// Full scale range
#define AFS_2G  0
#define AFS_4G  1
#define AFS_8G  2
#define AFS_16G 3

#define GFS_250DPS  0
#define GFS_500DPS  1
#define GFS_1000DPS 2
#define GFS_2000DPS 3

// Type def for function pointer
typedef int (*i2c_read_write_t)(char addr, char *data, char len);

// Read and write struct
struct i2c_device_t {
  i2c_read_write_t i2c_write;
  i2c_read_write_t i2c_read;
};

// IMU data structure
struct imu_t {
  float ax, ay, az, gx, gy, gz;
};

// Gyro calibration structure
struct gyro_cal_t {
  float x, y, z;
};

// Accelerometer calibration structure
struct accel_cal_t {
  float sx, sy, sz;
  float bx, by, bz;
};


class MPU9250 {
private:
  char _addr;
  i2c_device_t _i2c_dev;

  unsigned char data[14];

  int write2bytes(char byte0, char byte1);

public:
  MPU9250(char addr, i2c_device_t i2c_dev);

  // Functions
  bool initIMU();
  void readCalData();
  void readRawData();

  float getAres(int Ascale);
  float getGres(int Gscale);

  bool gyroCalibration(int numCalPoints = 1000);
  bool accelCalibration(int Ascale);

  void setGyroCalibration(gyro_cal_t gyro);
  void setAccelCalibration(accel_cal_t accel);

  gyro_cal_t getGyroCalibration();
  accel_cal_t getAccelCalibration();

  // Variables
  float _aRes, _gRes;

  imu_t imu_raw;
  imu_t imu_cal;

  gyro_cal_t gyro_cal;
  accel_cal_t accel_cal;

  int temperature;
};

#endif //MPU9250_H
