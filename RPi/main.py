import smbus
import math



# 250 deg/s --> 131, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
scaleFactorGyro = 65.5

# 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
scaleFactorAccel = 8192.0



################################################################################
## FUNCTIONS
################################################################################

# Reads high and low 8 bit values and bit shifts them into 16 bit
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

# Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val



################################################################################
## Set Up
################################################################################
bus = smbus.SMBus(1)
address = 0x68

# Activate the MPU-6050
bus.write_byte_data(address, 0x6B, 0x00)

# Configure the accelerometer
# 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
bus.write_byte_data(address, 0x1C, 0x08)

# Configure the gyro
# 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
bus.write_byte_data(address, 0x1B, 0x08)



################################################################################
## Reading
################################################################################

gyro_x = read_word_2c(0x43) / scaleFactorGyro
gyro_y = read_word_2c(0x45) / scaleFactorGyro
gyro_z = read_word_2c(0x47) / scaleFactorGyro

acc_x = read_word_2c(0x3b) / scaleFactorAccel
acc_y = read_word_2c(0x3d) / scaleFactorAccel
acc_z = read_word_2c(0x3f) / scaleFactorAccel



################################################################################
## Display
################################################################################
print "Gyroscope: "
print gyro_x
print gyro_y
print gyro_z

print "Accelerometer: "
print acc_x
print acc_y
print acc_z
