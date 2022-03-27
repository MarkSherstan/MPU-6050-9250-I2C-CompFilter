# MPU-6050 and MPU-9250 I2C Complementary Filter
Testing different methods to interface with a MPU-6050 or MPU-9250 via I2C and SPI. All methods feature the extraction of the raw sensor values as well as the implementation of a complementary filter for the fusion of the gyroscope and accelerometer to yield an angle(s) in 3 dimensional space.

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
| Uno	      | A4            | A5            |
| Mega2560	 | 20	       | 21            |
| Leonardo      | 2	            | 3             |
| Due           | 20	       | 21            |

Upload the `main.ino` sketch and observe the values in the serial port or serial plotter. The `calibrateGyro.ino` sketch can be used to retrieve the offset values which can be directly placed into the `main.ino` sketch to eliminate the need for calibration every time the microcontroller is started up. Note that this is at the cost of performance as the sensors drift over time and between uses.

### MATLAB
Connect an Arduino using the same wiring as outlined above. Run `MATLAB\I2C\main.m` and observe the values in the command line. MATLAB is extremely slow when using an Arduino/I2C connection.

A faster method is to read data through a serial connection. Using the same wiring connection, upload the sketch in `Visualizer\arduinoSketch` to the Arduino board. Adjust any desired parameters as outlined below in the `MATLAB\serial\main.m` file. Run and observe the values in the command line.  

```
% Set up the class
gyro = 250;                       % 250, 500, 1000, 2000 [deg/s]
acc = 2;                          % 2, 4, 7, 16 [g]
tau = 0.98;                       % Time constant
port = '/dev/cu.usbmodem14101';   % Serial port name
```

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

### 9 DOF MPU-9250 RPi (Python) Madgwick Filter
Follow the same setup guide as in the **RPi (Python)** section, however an MPU-9250 must be used.

The code is based on Kriswiner's C++ MPU-9250 library located [here](https://github.com/kriswiner/MPU9250) and Sebastian Madgwick's open source IMU and AHRS algorithms located [here](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/). To run the program navigate to the `\9DOF` directory and run `python3 main.py`.

Before running the program and sensor fusion algorithms the magnetometer must be calibrated. Uncomment the second line below and the program will walk you through all the required steps. Once values have been retrieved enter them in lines three and four below.  

```
# Calibrate the mag or provide values that have been verified with the visualizer
# mpu.calibrateMagGuide()
bias = [282.893, 300.464, -91.72]
scale = [1.014, 1.054, 0.939]
mpu.setMagCalibration(bias, scale)
```

To verify the results of the calibration refer to the two articles located [here](https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration) and [here](https://appelsiini.net/2018/calibrate-magnetometer/). Place the values from the calibration into `data.txt` and `magCalVisualizer.py` and `magCalSlider.py` as described in the program guide during calibration. The `magCalVisualizer.py` and `magCalSlider.py` script will provide all the required plots to aid in verifying the results as well as interactive sliders to optimize values.

Note that the magnetometer can either be read as a slave or in direct mode. Both methods exist with slave being the default mode. 

### Node.js and p5.js Visualizer
Connect an IMU device as outlined in the Arduino section. Upload the sketch located in `Visualizer/arduinoSketch`. This sketch simply transmits the raw byte data from the sensor over a serial connection.

Next, install the required packages by performing these steps:

```
cd MPU-6050-9250-I2C-CompFilter/render
npm install
```

Edit the file `main.js` file located in `Visualizer/render` to set the correct serial port and any other parameters of interest. 

```
// Customize these values
const serialPortName = 'COM4';
const serialBaud = 9600;
var tau = 0.98;
var gyroScaleFactor = 65.5;
var accScaleFactor = 8192.0;
var calibrationPts = 250;
```

Once all the values are customized start the serial port server by navigating to `Visualizer/render` and entering:

```
node main.js
```

In your default browser enter `localhost:3000` and the visualizer should be running. 

**Ensure to hold the IMU device still until an object appears on the screen. This is the program performing a calibration for gyroscope offset.**

### STM32
The following was tested with a [NUCLEO-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) dev board ([pinout](https://os.mbed.com/platforms/ST-Nucleo-F401RE/)). Using a similar configuration and copying the `MPUXX50.c` and `MPUXX50.h` files into their respective `\Src` and `\Inc` directories, and adjusting the `huart#`, `hi2c#`, and `hspix` values the base code should work for any STM32 based device. Addional set up for `TIM#` and `GPIO` pins may be required.
* Download [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) and create a new project based on hardware.
* Select: Project Manager -> Code Generator -> Check `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`.

For **I2C:**
* Configure the pins (Example uses `USART2`, `I2C1`, and `TIM11`). The serial port runs with default settings at a baud rate of 115200, the I2C port runs in standard mode at 100 kHz, the timer is set to interrupt at 250 Hz (adjust prescaler and counter based on hardware and clock speeds). Configure a GPIO port with an indicator LED if desired. 
* Use `MPU_begin(...)` to configure the IMU settings and ensure that there is a connection. The IMU should be calibrated with `MPU_calibrateGyro(...)`, and to retrieve attitude use `MPU_calcAttitude(...)`. See `main.c` for the full example implementation with additional notes.
* Code could use a little tidy (e.g. remove global variables, make it simpler to initialize, etc...)

For **SPI:**
* MPU6050 does not support SPI, a MPU9250 must be used (confirm hardware set up before use; some breakout boards require adjusting of solder jumpers to be used in SPI mode as there is overlap with I2C hardare e.g. AD0)
* Configure the pins (Example uses `USART2`, `SPI1`, `PB6` and `TIM11`). The serial port runs with default settings at a baud rate of 115200, the SPI port runs with a 128 prescaler to keep the rate below 1 MHz, the timer is set to interrupt at 250 Hz (adjust prescaler and counter based on hardware and clock speeds), and PB6 is set as a digital Chip Select pin with a high output level and high max output speed.
* Use `MPU_begin(...)` to configure the IMU settings and ensure that there is a connection. The IMU should be calibrated with `MPU_calibrateGyro(...)`, and to retrieve attitude use `MPU_calcAttitude(...)`. See `main.c` for the full example implementation with additional notes.
* Code is very similar to the I2C example; however, it makes more use of pointers and is cleaner. 

### C++ Library
A generic C++ library was written that can be used on a variety of hardware. Refer to the Arduino or Raspberry Pi example in the `CPP_library` directory to get an idea of how to use the library.

For the Arduino example ensure to add the library to your Arduino IDE or put the `mpuXX50.h` and `mpuXX50.cpp` in the same folder as your `*.ino`.

For the Raspberry Pi you may need to run the following commands before using the `Makefile`.

```
sudo apt-get install libi2c-dev
sudo apt-get install i2c-tools
sudo apt-get update

sudo i2cdetect -y 0
//or
sudo i2cdetect -y 1
```

Upon setting up the class with the I2C address of the sensor and defining the read and write functions the library has the capability to.
* Perform WHO_AM_I sensor self check
* Set the resolution of the accelerometer and gyroscope
* Perform, set, and return gyroscope calibration values
* Return raw sensor values, calibrated sensor values, and complementary fused values yielding sensor attitude - roll, pitch, and yaw (yaw will drift over time)

## Future Ideas
* Add quaternion angle representation
* Kalman filter (custom lib)
* C Library
* General clean up
* Move STM32 examples to their own repo (just put the libs)
