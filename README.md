# MPU-6050 and MPU-9250 I2C Complementary Filter
Testing different methods to interface with a MPU-6050 or MPU-9250 via I2C. All methods feature the extraction of the raw sensor values as well as the implementation of a complementary filter for the fusion of the gyroscope and accelerometer to yield an angle(s) in 3 dimensional space.

## Registry Maps and Sensitivity Values
Values retrieved below come from the MPU-6050 and MPU-9250 registry maps and product specifications documents located in the `\Resources` folder. Configure the gyroscope on `0x1B` and the accelerometer on `0x1C` as per data sheets with the following values (the MPU-6050 and MPU-9250 are interchangeable and all registries are the same):

| Accelerometer | Sensitivity   | Gyroscope     | Sensitivity   | Hexadecimal   |  Binary       |
| ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| +/- 2g	      | 16384	        | +/- 250 deg/s | 131           | 0x00	        | 00000000      |
| +/- 4g	      | 8192 	        | +/- 500 deg/s | 65.5          | 0x08	        | 00001000      |
| +/- 8g        | 4096	        | +/- 1000 deg/s| 32.8          | 0x10	        | 00010000      |
| +/- 16g	      | 2048	        | +/- 2000 deg/s| 16.4          | 0x18	        | 00011000      |

The slave address is b110100X which is 7 bits long. The LSB bit of the 7 bit address is determined by the logic level on pin AD0. This allows two sensors to be connected to the same I2C bus. When used in this configuration, the address of one of the devices should be b1101000 (pin AD0 is logic low) and the address of the other should be b1101001 (pin AD0 is logic high). Communication will typically take place over the `0x68` register.

**Ensure that the proper logic (3.3V vs 5V) is being used so you do not fry your sensor**

## Use
### Arduino
Connect the sensor to the microcontroller as outlined below.

* VCC --> 5V or 3.3V based on specific breakout board and logic levels
* GND --> GND
* SDA and SCL pins located in table below:

| Board         | SDA Pin       | SCL Pin       |
| ------------- | ------------- | ------------- |
| Uno	          | A4            | A5            |
| Mega2560	    | 20	          | 21            |
| Leonardo      | 2	            | 3             |
| Due           | 20	          | 21            |

Upload the `main.ino` sketch and observe the values in the serial port or serial plotter. The `calibrateGyro.ino` sketch can be used to retrieve the offset values which can be directly placed into the `main.ino` sketch to eliminate the need for calibration every time the microcontroller is started up. Note that this is at at the cost of performance as the sensors drift over time and between uses.

### MATLAB
Connect an Arduino using the same wiring as outlined above. Run `main.m` and observe the values in the command line. MATLAB is extremely slow with I2C devices through an Arduino and it will be faster to run a serial connection with the data acquisition occurring on a microcontroller (see the Visualizer section).

### RPi (Python)
Connect your IMU sensor to 5V or 3.3V based on specific breakout board and ground to ground. Refer to the pinout of your board using [pinout.xyz](https://pinout.xyz) and match SDA and SCL accordingly.

Setup as described [here](https://tutorials-raspberrypi.com/measuring-rotation-and-acceleration-raspberry-pi/). First enter `sudo raspi-config` and enable SPI and I2C. Reboot the Pi.

Edit the modules file using `sudo nano /etc/modules` and ensure `i2c-bcm2708` and `i2c-dev` are both in the file. Reboot again.

With the sensor correctly wired enter the following in the command line.
```
sudo apt-get install i2c-tools python-smbus
sudo i2cdetect -y 1
```

Which should yield the table below (possible to have the value 0x69) verifying a proper connection:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --  
```
Once verified run `python3 main.py` to observe the values.

### JavaScript Visualizer
Connect an IMU device as outlined in the Arduino section. Upload the sketch located in `Visualizer/arduinoSketch`. This sketch simply transmits the raw byte data from the sensor over a serial connection.

Next, perform the following commands to add the necessary server which makes a bridge between the microcontroller and web application.  

```
cd MPU-6050-9250-I2C-CompFilter/
git submodule add https://github.com/vanevery/p5.serialport.git Visualizer/p5js/server
cd Visualizer/p5js/server/
npm install
```

Edit the file `sketch.js` file located in `Visualizer/p5js/main` to set the desired properties and correct serial port.

```
// Customize these values
var portName = '/dev/cu.usbmodem14101';
var tau = 0.98;
var gyroScaleFactor = 65.5;
var accScaleFactor = 8192.0;
var calibrationPts = 100;
```

Once all the values are customized start the serial port server by navigating to `Visualizer/p5js/server/` and entering:

```
node startserver.js
```

Double click on the `index.html` file located in `Visualizer/p5js/main` and the program will begin in your default browser. Alternatively you can enter the file path in a browser as such `file:///Users/MarkSherstan/Documents/GitHub/MPU-6050-9250-I2C-CompFilter/Visualizer/p5js/main/index.html`

**Ensure to hold the IMU device still until an object appears on the screen. This is the program performing a calibration for gyroscope offset.**

### MATLAB Visualizer
Connect an IMU device as outlined in the Arduino section. Upload the sketch located in `Visualizer/arduinoSketch`. This sketch simply transmits the raw byte data from the sensor over a serial connection.


## Future Ideas
* RPi C++ Version
* Add quaternion angle representation
