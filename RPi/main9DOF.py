import smbus
import math
import time

class MPU:
    def __init__(self, gyro, acc, tau):
        # Class / object / constructor setup
        self.gx = None; self.gy = None; self.gz = None;
        self.ax = None; self.ay = None; self.az = None;
        self.mx = None; self.my = None; self.mz = None;

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

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

        self.bus = smbus.SMBus(1)
        self.IMUaddress = 0x68
        self.MAGaddress = 0x0C

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

    def setUp(self):
        # Activate the MPU
        self.bus.write_byte_data(self.IMUaddress, 0x6B, 0x00)

        # Configure the accelerometer
        self.bus.write_byte_data(self.IMUaddress, 0x1C, self.accHex)

        # Configure the gyro
        self.bus.write_byte_data(self.IMUaddress, 0x1B, self.gyroHex)

        # Configure the mag for 16 bits and continous mode 2 (100 Hz ?)
        self.bus.write_byte_data(self.MAGaddress, 0x0A, 0x16)

        # Display message to user
        print("MPU set up:")
        print('\tAccelerometer: ' + str(self.accHex) + ' ' + str(self.accScaleFactor))
        print('\tGyro: ' + str(self.gyroHex) + ' ' + str(self.gyroScaleFactor) + "\n")
        time.sleep(2)

    def eightBit2sixteenBitIMU(self, reg):
        # Reads high and low 8 bit values and shifts them into 16 bit
        h = self.bus.read_byte_data(self.IMUaddress, reg)
        l = self.bus.read_byte_data(self.IMUaddress, reg+1)
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def eightBit2sixteenBitMAG(self, reg):
        # Reads low and high 8 bit values and shifts them into 16 bit
        l = self.bus.read_byte_data(self.IMUaddress, reg)
        h = self.bus.read_byte_data(self.IMUaddress, reg+1)
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def getRawData(self):
        self.gx = self.eightBit2sixteenBitIMU(0x43)
        self.gy = self.eightBit2sixteenBitIMU(0x45)
        self.gz = self.eightBit2sixteenBitIMU(0x47)

        self.ax = self.eightBit2sixteenBitIMU(0x3B)
        self.ay = self.eightBit2sixteenBitIMU(0x3D)
        self.az = self.eightBit2sixteenBitIMU(0x3F)

        self.mx = self.eightBit2sixteenBitMAG(0x03)
        self.my = self.eightBit2sixteenBitMAG(0x05)
        self.mz = self.eightBit2sixteenBitMAG(0x07)

    def calibrateGyro(self, N):
        # Display message
        print("Calibrating gyro with " + str(N) + " points. Do not move!")

        # Take N readings for each coordinate and add to itself
        for ii in range(N):
            self.getRawData()
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
        time.sleep(2)
        self.dtTimer = time.time()

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
    MPU9250_ADDRESS  = 0x68
    AK8963_ADDRESS   = 0x0C

    USER_CTRL        = 0x6A
    I2C_MST_CTRL     = 0x24
    I2C_SLV0_ADDR    = 0x25
    I2C_SLV0_REG     = 0x26
    I2C_SLV0_CTRL    = 0x27
    WHO_AM_I_AK8963  = 0x00
    EXT_SENS_DATA_00 = 0x49

    bus = smbus.SMBus(1)

    print("Who am I IMU? Should be: " + str(0x71))
    print(bus.read_byte_data(0x68, 0x75)) # Who am I -> should return 0x71


    bus.write_byte_data(MPU9250_ADDRESS, USER_CTRL, 0x20);    # Enable I2C Master mode
    bus.write_byte_data(MPU9250_ADDRESS, I2C_MST_CTRL, 0x0D); # I2C configuration multi-master I2C 400KHz

    bus.write_byte_data(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
    bus.write_byte_data(MPU9250_ADDRESS, I2C_SLV0_REG, WHO_AM_I_AK8963);           # I2C slave 0 register address from where to begin data transfer
    bus.write_byte_data(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     # Enable I2C and transfer 1 byte

    time.sleep(1)

    print("Who am I MAG? Should be: " + str(0x48))
    print(bus.read_byte_data(MPU9250_ADDRESS, EXT_SENS_DATA_00)) # Who am I -> should return 0x48

    # # Set up class
    # gyro = 250      # 250, 500, 1000, 2000 [deg/s]
    # acc = 2         # 2, 4, 7, 16 [g]
    # tau = 0.98
    # mpu = MPU(gyro, acc, tau)
    #
    # # Set up sensor and calibrate gyro with N points
    # mpu.setUp()
    # mpu.calibrateGyro(500)
    #
    # # Run for 20 secounds
    # startTime = time.time()
    # while(time.time() < (startTime + 20)):
    #     mpu.compFilter()
    #
    # # End
    # print("Closing")


# MFS_16BITS == 0.15 mG per LSB      Mscale = MFS_16BITS
# M}mode = M_100Hz           Mmode = M_100Hz

# int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
# float   magCalibration[3] = {0, 0, 0};

# These can be measured once and entered here or can be calculated each time the device is powered on
# float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


# Main loop
if __name__ == '__main__':
	main()
