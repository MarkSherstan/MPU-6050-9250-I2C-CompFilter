# MPU6050_Visualizer
Test project using MATLAB's Arduino package to interface with an I2C sensor device and the use of a complementary filter.


## Registry Maps and Sensitivity Values for MPU-6050

Values retrieved from the MPU-6050 Register Map and Descriptions Revision 4.2 located [here](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) and the Product Specification Revision 3.4 located [here](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).

Configure the gyroscope on 0x1B and the accelerometer on 0x1C as per data sheets with the following values:

| Accelerometer | Sensitivity   | Gyroscope     | Sensitivity   | Hexadecimal   |  Binary       |
| ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| +/- 2g	      | 16384	        | 250 deg/s     | 131           | 0x00	        | 00000000      |
| +/- 4g	      | 8192 	        | 500 deg/s     | 65.5          | 0x08	        | 00001000      |
| +/- 8g        | 4096	        | 1000 deg/s    | 32.8          | 0x10	        | 00010000      |
| +/- 16g	      | 2048	        | 2000 deg/s    | 16.4          | 0x18	        | 00011000      |


It should also be noted that the pin "AD0" selects between I2C address 0x68 and 0x69 which allows for multiple sensors. Primary communication will take place over the 0x68 register. Further resources can be found [here](https://playground.arduino.cc/Main/MPU-6050).


## Connections

* VCC --> 5V or 3.3V based on specific breakout Board
* GND --> GND
* SDA and SCL pins located in table below:

| Board         | SDA Pin       | SCL Pin       |
| ------------- | ------------- | ------------- |
| Uno	          | A4            | A5            |
| Mega2560	    | 20	          | 21            |
| Leonardo      | 2	            | 3             |
| Due           | 20	          | 21            |

## Usage

Run in the following order:

```
arduinoConnection.m

main(ard,dev,20,0.99)
```

Where ard and dev are saved from arduinoConnection.m and totalTime is defined by the user in seconds.
