//Include I2C library and declare some variables
#include <Wire.h>

int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;


// Run Once
void setup() {
  Wire.begin();
  Serial.begin(57600);

  //Setup the registers of the MPU-6050 and start up
  setup_mpu_6050_registers();

  // Display Instructions
  Serial.println("MPU-6050 IMU Gyro Calibration");

  delay(1500);

  Serial.println("Calibrating gyro, place on level surface and do not move.");
  Serial.println();
  Serial.println("Taking 5000 readings");

  delay(2500);

  // Take 5000 readings for each coordinate and then find average offset
  for (int cal_int = 0; cal_int < 5000 ; cal_int ++){
    if(cal_int % 200 == 0)Serial.print(".");
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    delay(4);   //Delay 4ms to simulate 250Hz loop, f = 1/T - Assume 0 calc time
  }

  // Average the values
  gyro_x_cal /= 5000;
  gyro_y_cal /= 5000;
  gyro_z_cal /= 5000;

  // Display results
  Serial.println("Complete");

  Serial.println();
  Serial.println("Place these values in the main program");
  Serial.println();

  Serial.print("gyro_x_cal: ");
  Serial.println(gyro_x_cal);

  Serial.print("gyro_y_cal: ");
  Serial.println(gyro_y_cal);

  Serial.print("gyro_z_cal: ");
  Serial.println(gyro_z_cal);
}


void loop() {
}


void read_mpu_6050_data(){
  //Subroutine for reading the raw gyro data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  // Read data
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //Configure the gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00); // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.endTransmission();
}
