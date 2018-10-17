# MPU6050_Visualizer
Test project using MATLAB's Arduino package to interface with an I2C sensor device. 


## Registry Maps and Sensitivity Values for MPU-6050

Values retrieved from the MPU-6050 Register Map and Descriptions Revision 4.2 located [here](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) and the Product Specification Revision 3.4 located [here](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).

Configure the gyroscope on 0x1B and the accelerometer on 0x1C as per data sheets with the following values:

| Accelerometer | Sensitivity   | Gyroscope     | Sensitivity   | Hexadecimal   |  Binary       |
| ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| +/- 2g	      | 16384	        | 250 deg/s     | 131           | 0x00	        | 00000000      |
| +/- 4g	      | 8192 	        | 500 deg/s     | 65.5          | 0x08	        | 00001000      |
| +/- 8g        | 4096	        | 1000 deg/s    | 32.8          | 0x10	        | 00010000      |
| +/- 16g	      | 2048	        | 2000 deg/s    | 16.4          | 0x18	        | 00011000      |

Read values at 0x3B and request 14 values (high and low) to retireve all gyroscope and accelerometer bytes. 0x41 and 0x42 are temperature so neglect values 7 and 8 or request 6 values starting at 0x3B and 6 more values starting at 0x43.

It should also be noted that the pin "AD0" selects between I2C address 0x68 and 0x69 which allows for multiple sensors. Primary communcation will take place over the 0x68 register. Further resources can be found [here](https://playground.arduino.cc/Main/MPU-6050).


## References to Read

https://www.mathworks.com/help/supportpkg/arduinoio/examples/measure-temperature-from-i2c-device-on-arduino-hardware.html?prodcode=ML

https://www.mathworks.com/matlabcentral/answers/333946-how-to-plot-the-real-time-data-from-arduino-in-matlab

https://www.mathworks.com/videos/plotting-live-data-of-a-temperature-sensor-using-arduino-and-matlab-121317.html
