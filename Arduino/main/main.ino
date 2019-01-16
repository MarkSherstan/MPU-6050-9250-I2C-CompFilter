//Include I2C library and declare variables
#include <Wire.h>

int gyro_x, gyro_y, gyro_z;
long loop_timer;
long scaleFactorGyro = 65.5; // 250 deg/s --> 131, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
double rotation_x, rotation_y, rotation_z;

long acc_x, acc_y, acc_z;
int temperature;
long scaleFactorAccel = 8192; // 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
double accel_x, accel_y, accel_z;


// Set offsets from initial calibration
long gyro_x_cal = -213;
long gyro_y_cal = 240;
long gyro_z_cal = -297;


// Run once
void setup() {
  Wire.begin();

  Serial.begin(57600);

  //Setup the registers of the MPU-6050 and start up
  setup_mpu_6050_registers();

  //Reset the loop timer
  loop_timer = micros();
}


void loop() {
  // Read the raw acc data from MPU-6050
  read_mpu_6050_data();

  // Subtract the offset calibration value
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Convert to instantaneous degrees per second
  rotation_x = (double)gyro_x / (double)scaleFactorGyro;
  rotation_y = (double)gyro_y / (double)scaleFactorGyro;
  rotation_z = (double)gyro_z / (double)scaleFactorGyro;

  // Convert to g force
  accel_x = (double)acc_x / (double)scaleFactorAccel;
  accel_y = (double)acc_y / (double)scaleFactorAccel;
  accel_z = (double)acc_z / (double)scaleFactorAccel;

  // Print / write data
  Serial.print(loop_timer); Serial.print(",");
  Serial.print(accel_x, 7); Serial.print(",");
  Serial.print(accel_y, 7); Serial.print(",");
  Serial.print(accel_z, 7); Serial.print(",");
  Serial.print(rotation_x, 7); Serial.print(",");
  Serial.print(rotation_y, 7); Serial.print(",");
  Serial.println(rotation_z, 7);


  // Wait until the loop_timer reaches 4000us (250Hz) before next loop
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}


void read_mpu_6050_data() {
  //Subroutine for reading the raw accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  // Read data --> Temperature falls between acc and gyro registers
  while(Wire.available() < 14);
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() <<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
}


void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //Configure the accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08); // 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.endTransmission();

  //Configure the gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08); // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.endTransmission();
}
