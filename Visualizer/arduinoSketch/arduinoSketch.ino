//Include I2C library and declare variables
#include <Wire.h>

int acc_x, acc_y, acc_z;
int gyro_x, gyro_y, gyro_z;
int temperature;


void setup() {
  // Start
  Wire.begin();
  Serial.begin(9600);

  // Setup the registers of the MPU-6050 and start up
  setup_mpu_6050_registers();
}


void loop() {
  // Read the raw data from MPU
  read_mpu_6050_data();

  // Write to the serial port in bytes
  writeBytes(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
}


void writeBytes(int* data1, int* data2, int* data3, int* data4, int* data5, int* data6){
  // Cast to a byte pointer(s)
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);

  // Byte array with header for transmission
  byte buf[14] = {0x9F, 0x6E,
                byteData1[0], byteData1[1],
                byteData2[0], byteData2[1],
                byteData3[0], byteData3[1],
                byteData4[0], byteData4[1],
                byteData5[0], byteData5[1],
                byteData6[0], byteData6[1]};

  // Write the byte(s)
  Serial.write(buf, 14);
}


void read_mpu_6050_data() {
  // Subroutine for reading the raw data
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

  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}
