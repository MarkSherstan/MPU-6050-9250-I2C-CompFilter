import smbus
import math
import time

class MPU:
    def __init__(self, gyro, acc, mag, tau):
        # Class / object / constructor setup
        self.gx = None; self.gy = None; self.gz = None;
        self.ax = None; self.ay = None; self.az = None;
        self.mx = None; self.my = None; self.mz = None;

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

        self.magXcal = 0
        self.magYcal = 0
        self.magZcal = 0

        self.magXbias = 0
        self.magYbias = 0
        self.magZbias = 0

        self.magXscale = 0
        self.magYscale = 0
        self.magZscale = 0

        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.dtTimer = 0
        self.tau = tau

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor, self.accHex = self.accelerometerSensitivity(acc)
        self.magScaleFactor, self.magHex = self.magnetometerSensitivity(mag)

        self.bus = smbus.SMBus(1)

        self.MPU9250_ADDRESS  = 0x68
        self.AK8963_ADDRESS   = 0x0C

        self.WHO_AM_I_MPU9250 = 0x75
        self.WHO_AM_I_AK8963  = 0x00

        self.AK8963_CNTL      = 0x0A
        self.AK8963_CNTL2     = 0x0B
        self.I2C_SLV0_DO      = 0x63
        self.USER_CTRL        = 0x6A
        self.I2C_MST_CTRL     = 0x24
        self.I2C_SLV0_ADDR    = 0x25
        self.I2C_SLV0_REG     = 0x26
        self.I2C_SLV0_CTRL    = 0x27
        self.EXT_SENS_DATA_00 = 0x49
        self.PWR_MGMT_1       = 0x6B
        self.ACCEL_CONFIG     = 0x1C
        self.GYRO_CONFIG      = 0x1B
        self.AK8963_ASAX      = 0x10
        self.ACCEL_XOUT_H     = 0x3B
        self.AK8963_XOUT_L    = 0x03

    def gyroSensitivity(self, x):
        # Create dictionary with standard value of 500 deg/s
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x):
        # Create dictionary with standard value of 4 g
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0,  0x18]
        }.get(x,[8192.0,  0x08])

    def magnetometerSensitivity(self, x):
        # Create dictionary with standard value of 16 bit
        return {
            14:  [10.0*4912.0/8190.0,  0x06],
            16:  [10.0*4912.0/32760.0, 0x16],
        }.get(x,[10.0*4912.0/32760.0,  0x16])

    def setUpIMU(self):
        # Check to see if there is a good connection with the MPU 9250
        whoAmI = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.WHO_AM_I_MPU9250)

        if (whoAmI == 0x71):
            # Connection is good! Activate/reset the IMU
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.PWR_MGMT_1, 0x00)

            # Configure the accelerometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.ACCEL_CONFIG, self.accHex)

            # Configure the gyro
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.GYRO_CONFIG, self.gyroHex)

            # Display message to user
            print("MPU set up:")
            print('\tAccelerometer: ' + str(hex(self.accHex)) + ' ' + str(self.accScaleFactor))
            print('\tGyroscope: ' + str(hex(self.gyroHex)) + ' ' + str(self.gyroScaleFactor) + "\n")
        else:
            # Bad connection or something went wrong
            print("IMU WHO_AM_I was: " + hex(whoAmI) + ". Should have been " + hex(0x71))

    def setUpMAG(self):
        # Initialize connection with mag for a WHO_AM_I test
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.USER_CTRL, 0x20);                              # Enable I2C Master mode
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_MST_CTRL, 0x0D);                           # I2C configuration multi-master I2C 400KHz
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.WHO_AM_I_AK8963);           # I2C slave 0 register address from where to begin data transfer
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                          # Enable I2C and transfer 1 byte
        time.sleep(0.05)

        # Check to see if there is a good connection with the mag
        whoAmI = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00)

        if (whoAmI == 0x48):
            # Connection is good! Begin the true initialization
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL2);        # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x01);                      # Reset AK8963
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x00);                      # Power down magnetometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x0F);                      # Enter fuze mode
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);   # Set the I2C slave address of AK8963 and set for read.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_ASAX);              # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x83);                         # Enable I2C and read 3 bytes
            time.sleep(0.05)

            # Read the x, y, and z axis calibration values
            rawData = []
            for ii in range(3):
                rawData.append(self.bus.read_byte_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00 + ii))

            # Convert values to something more usable
            self.magXcal =  float(rawData[0] - 128)/256.0 + 1.0;
            self.magYcal =  float(rawData[1] - 128)/256.0 + 1.0;
            self.magZcal =  float(rawData[2] - 128)/256.0 + 1.0;

            # Configure the settings for the mag
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, self.magHex);               # Set magnetometer for 14 or 16 bit continous 100 Hz sample rates
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);               # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                          # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            # Display results to user
            print("MAG set up:")
            print("\tMagnetometer: " + hex(self.magHex) + " " + str(round(self.magScaleFactor,3)) + "\n")
        else:
            # Bad connection or something went wrong
            print("MAG WHO_AM_I was: " + hex(whoAmI) + ". Should have been " + hex(0x48))

    def eightBit2sixteenBit(self, l, h):
        # Shift the low and high byte into a 16 bit number
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def readRawIMU(self):
        # Read 14 raw values [High Low] as temperature falls between the accelerometer and gyro registries
        rawData = []
        for ii in range(14):
            rawData.append(self.bus.read_byte_data(self.MPU9250_ADDRESS, self.ACCEL_XOUT_H + ii))

        # Convert the raw values to something a little more useful
        self.ax = self.eightBit2sixteenBit(rawData[1], rawData[0])
        self.ay = self.eightBit2sixteenBit(rawData[3], rawData[2])
        self.az = self.eightBit2sixteenBit(rawData[5], rawData[4])

        self.gx = self.eightBit2sixteenBit(rawData[7], rawData[6])
        self.gy = self.eightBit2sixteenBit(rawData[9], rawData[8])
        self.gz = self.eightBit2sixteenBit(rawData[11], rawData[10])

    def readRawMag(self):
        # Prepare to request values
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_XOUT_L);             # I2C slave 0 register address from where to begin data transfer
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x87);                          # Enable I2C and read 7 bytes
        time.sleep(0.02)

        # Read 7 values [Low High] and one more byte (overflow check)
        rawData = []
        for ii in range(7):
            rawData.append(self.bus.read_byte_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00 + ii))

        # If overflow check passes convert the raw values to something a little more useful
        if not (rawData[6] & 0x08):
            self.mx = self.eightBit2sixteenBit(rawData[0], rawData[1])
            self.my = self.eightBit2sixteenBit(rawData[2], rawData[3])
            self.mz = self.eightBit2sixteenBit(rawData[4], rawData[5])

    def calibrateGyro(self, N):
        # Display message
        print("Calibrating gyro with " + str(N) + " points. Do not move!")

        # Take N readings for each coordinate and add to itself
        for ii in range(N):
            self.readRawIMU()
            self.gyroXcal += self.gx
            self.gyroYcal += self.gy
            self.gyroZcal += self.gz

        # Find average offset value
        self.gyroXcal /= N
        self.gyroYcal /= N
        self.gyroZcal /= N

        # Display message and restart timer for comp filter
        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXcal,1)))
        print("\tY axis offset: " + str(round(self.gyroYcal,1)))
        print("\tZ axis offset: " + str(round(self.gyroZcal,1)) + "\n")

        # Start the timer
        self.dtTimer = time.time()

    def calibrateMag(self):
        # Local calibration variables
        magBias = [0, 0, 0]
        magScale = [0, 0, 0]
        magMin = [32767, 32767, 32767]
        magMax = [-32767, -32767, -32767]
        magTemp = [0, 0, 0]

        # Display message
        print("Mag calibration! Wave device in a figure eight until done (~18 seconds).")
        time.sleep(3)

        # Take approx 15 seconds of mag data as we are sampling at 100 hz
        for ii in range(1500):
            # Read fresh values and assign to magTemp
            self.readRawMag()
            magTemp = [self.mx, self.my, self.mz]

            # Adjust the max and min points based off of current reading
            for jj in range(3):
                if (magTemp[jj] > magMax[jj]):
                    magMax[jj] = magTemp[jj]
                if (magTemp[jj] < magMin[jj]):
                    magMin[jj] = magTemp[jj]

            # Small delay before next loop (data available every 10 ms or 100 Hz)
            time.sleep(0.012)

        # Get hard iron correction
        self.magXbias = (magMax[0] + magMin[0])/2
        self.magYbias = (magMax[1] + magMin[1])/2
        self.magZbias = (magMax[2] + magMin[2])/2

        # Get soft iron correction estimate
        self.magXscale = (magMax[0] - magMin[0])/2
        self.magYscale = (magMax[1] - magMin[1])/2
        self.magZscale = (magMax[2] - magMin[2])/2

        # Display results to user
        print("Calibration complete")
        print("\tX Bias: " + str(self.magXbias))
        print("\tY Bias: " + str(self.magYbias))
        print("\tZ Bias: " + str(self.magZbias))
        print()
        print("\tX Scale: " + str(self.magXscale))
        print("\tY Scale: " + str(self.magYscale))
        print("\tZ Scale: " + str(self.magZscale))
        time.sleep(5)

    def processIMUvalues(self):
        # Update the raw data
        self.getRawData()

        # Subtract the offset calibration values
        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        # Convert to instantaneous degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        # Convert to g force
        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

    def compFilter(self):
        # Get the processed values from IMU
        self.processIMUvalues()

        # Get delta time and record time for next call
        dt = time.time() - self.dtTimer
        self.dtTimer = time.time()

        # Acceleration vector angle
        accPitch = math.degrees(math.atan2(self.ay, self.az))
        accRoll = math.degrees(math.atan2(self.ax, self.az))

        # Gyro integration angle
        self.gyroRoll -= self.gy * dt
        self.gyroPitch += self.gx * dt
        self.gyroYaw += self.gz * dt
        self.yaw = self.gyroYaw

        # Comp filter
        self.roll = (self.tau)*(self.roll - self.gy*dt) + (1-self.tau)*(accRoll)
        self.pitch = (self.tau)*(self.pitch + self.gx*dt) + (1-self.tau)*(accPitch)

        # Print data
        print(" R: " + str(round(self.roll,1)) \
            + " P: " + str(round(self.pitch,1)) \
            + " Y: " + str(round(self.yaw,1)))

def main():
    # Set up class
    gyro = 250      # 250, 500, 1000, 2000 [deg/s]
    acc = 2         # 2, 4, 7, 16 [g]
    mag = 16        # 14, 16 [bit]
    tau = 0.98
    mpu = MPU(gyro, acc, mag, tau)

    # Set up the sensors
    mpu.setUpIMU()
    mpu.setUpMAG()

    # Calibrate the gyro and/or mag
    # mpu.calibrateGyro(500)
    # mpu.calibrateMag()

    # # Run for 10 secounds
    startTime = time.time()
    while(time.time() < (startTime + 10)):
        mpu.readRawIMU()
        mpu.readRawMag()

    # End
    print("Closing")

# Main loop
if __name__ == '__main__':
	main()
