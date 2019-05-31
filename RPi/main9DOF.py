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

		self.q = [1,0,0,0]
		self.beta = 1


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

	def calibrateMag(self, state):
		# Local calibration variables
		magBias = [0, 0, 0]
		magScale = [0, 0, 0]
		magMin = [32767, 32767, 32767]
		magMax = [-32767, -32767, -32767]
		magTemp = [0, 0, 0]

		# Take approx 15 seconds of mag data as we are sampling at 100 hz
		for ii in range(1501):
			# Read fresh values and assign to magTemp
			self.readRawMag()
			magTemp = [self.mx, self.my, self.mz]

			# Adjust the max and min points based off of current reading
			for jj in range(3):
				if (magTemp[jj] > magMax[jj]):
					magMax[jj] = magTemp[jj]
				if (magTemp[jj] < magMin[jj]):
					magMin[jj] = magTemp[jj]

			# Display some info to the user depending on the state
			if (state == 0):
				# Print percent completion every 150 iteraitions to the user
				if (ii % 150 == 0):
					print(str(round((ii/1500)*100)) + ' % complete')
			elif (state == 1):
				# Print raw values
				print(str(self.mx)+','+str(self.my)+','+str(self.mz))
			else:
				# Display error
				print("State unkown!")

			# Small delay before next loop (data available every 10 ms or 100 Hz)
			time.sleep(0.012)

		# Get hard iron correction
		self.magXbias = ((magMax[0] + magMin[0])/2) * self.magScaleFactor * self.magXcal
		self.magYbias = ((magMax[1] + magMin[1])/2) * self.magScaleFactor * self.magYcal
		self.magZbias = ((magMax[2] + magMin[2])/2) * self.magScaleFactor * self.magZcal

		# Get soft iron correction estimate
		magXchord = (magMax[0] - magMin[0])/2
		magYchord = (magMax[1] - magMin[1])/2
		magZchord = (magMax[2] - magMin[2])/2

		avgChord = (magXchord + magYchord + magZchord)/3

		self.magXscale = avgChord/magXchord
		self.magYscale = avgChord/magYchord
		self.magZscale = avgChord/magZchord

	def calibrateMagGuide(self):
		# Display message
		print("Magnetometer calibration. Wave and rotate device in a figure eight until notified.\n")
		time.sleep(3)

		# Run the first calibration
		self.calibrateMag(0)

		# Display results to user
		print("\nCalibration complete:")
		print("\tmagXbias = " + str(round(self.magXbias,3)))
		print("\tmagYbias = " + str(round(self.magYbias,3)))
		print("\tmagZbias = " + str(round(self.magZbias,3)) + "\n")

		print("\tmagXscale = " + str(round(self.magXscale,3)))
		print("\tmagYscale = " + str(round(self.magYscale,3)))
		print("\tmagZscale = " + str(round(self.magZscale,3)) + "\n")

		# Give more instructions to the user
		print("Place above values in magCalVisualizer.py")
		print("Recording additional 1501 data points to verify the calibration")
		print("Repeat random figure eight pattern and rotations...\n")
		time.sleep(3)

		# Run the scecond calibration
		self.calibrateMag(1)

		# Provide final instructions
		print("\nCopy the raw values into data.txt")
		print("Run magCalVisualizer.py to validate calibration success")
		print("See the README for more information")
		print("Also compare the second calibration to the first:")

		# Display the second results
		print("\tmagXbias = " + str(round(self.magXbias,3)))
		print("\tmagYbias = " + str(round(self.magYbias,3)))
		print("\tmagZbias = " + str(round(self.magZbias,3)) + "\n")

		print("\tmagXscale = " + str(round(self.magXscale,3)))
		print("\tmagYscale = " + str(round(self.magYscale,3)))
		print("\tmagZscale = " + str(round(self.magZscale,3)) + "\n")

		# End program
		print("Terminating program now!")
		quit()

	def setMagCalibration(self, bias, scale):
		# Set the bias variables in all 3 axis
		self.magXbias = bias[0]
		self.magYbias = bias[1]
		self.magZbias = bias[2]

		# Set the scale variables in all 3 axis
		self.magXscale = scale[0]
		self.magYscale = scale[1]
		self.magZscale = scale[2]

	def processValues(self):
		# Update the raw data
		self.readRawIMU()
		self.readRawMag()

		# Subtract the offset calibration values for the gyro
		self.gx -= self.gyroXcal
		self.gy -= self.gyroYcal
		self.gz -= self.gyroZcal

		# Convert the gyro values to degrees per second
		self.gx /= self.gyroScaleFactor
		self.gy /= self.gyroScaleFactor
		self.gz /= self.gyroScaleFactor

		# Convert the accelerometer values to g force
		self.ax /= self.accScaleFactor
		self.ay /= self.accScaleFactor
		self.az /= self.accScaleFactor

		# Process mag values in milliGauss
		# Include factory calibration per data sheet and user environmental corrections
		self.mx = self.mx * self.magScaleFactor * self.magXcal - self.magXbias
		self.my = self.my * self.magScaleFactor * self.magYcal - self.magYbias
		self.mz = self.mz * self.magScaleFactor * self.magZcal - self.magZbias

		self.mx *= self.magXscale
		self.my *= self.magYscale
		self.mz *= self.magZscale

	def compFilter(self):
		# Get the processed values from IMU and mag
		self.processValues()

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

		# Comp filter
		self.roll = (self.tau)*(self.roll - self.gy*dt) + (1-self.tau)*(accRoll)
		self.pitch = (self.tau)*(self.pitch + self.gx*dt) + (1-self.tau)*(accPitch)

	def madgwickFilter(self, ax, ay, az, gx, gy, gz, mx, my, mz, deltat):
	  # Quaternion values
	  q1 = self.q[0]
	  q2 = self.q[1]
	  q3 = self.q[2]
	  q4 = self.q[3]

	  # Auxiliary variables
	  q1x2 = 2 * q1
	  q2x2 = 2 * q2
	  q3x2 = 2 * q3
	  q4x2 = 2 * q4
	  q1q3x2 = 2 * q1 * q3
	  q3q4x2 = 2 * q3 * q4
	  q1q1 = q1 * q1
	  q1q2 = q1 * q2
	  q1q3 = q1 * q3
	  q1q4 = q1 * q4
	  q2q2 = q2 * q2
	  q2q3 = q2 * q3
	  q2q4 = q2 * q4
	  q3q3 = q3 * q3
	  q3q4 = q3 * q4
	  q4q4 = q4 * q4

	  # Normalize accelerometer measurement
	  norm = math.sqrt(ax * ax + ay * ay + az * az)
	  if norm is 0: return
	  ax /= norm
	  ay /= norm
	  az /= norm

	  # Normalize magnetometer measurement
	  norm = math.sqrt(mx * mx + my * my + mz * mz)
	  if norm is 0: return
	  mx /= norm
	  my /= norm
	  mz /= norm

	  # Reference direction of Earth's magnetic field
	  hx = mx * q1q1 - (2*q1*my) * q4 + (2*q1*mz) * q3 + mx * q2q2 + q2x2 * my * q3 + q2x2 * mz * q4 - mx * q3q3 - mx * q4q4
	  hy = (2*q1*mx) * q4 + my * q1q1 - (2*q1*mz) * q2 + (2*q2*mx) * q3 - my * q2q2 + my * q3q3 + q3x2 * mz * q4 - my * q4q4
	  bx_2 = math.sqrt(hx * hx + hy * hy)
	  bz_2 = -(2*q1*mx) * q3 + (2*q1*my) * q2 + mz * q1q1 + (2*q2*mx) * q4 - mz * q2q2 + q3x2 * my * q4 - mz * q3q3 + mz * q4q4
	  bx_4 = 2 * bx_2
	  bz_4 = 2 * bz_2

	  # Gradient decent algorithm corrective step
	  s1 = -q3x2 * (2 * q2q4 - q1q3x2 - ax) + q2x2 * (2 * q1q2 + q3q4x2 - ay) - bz_2 * q3 * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (-bx_2 * q4 + bz_2 * q2) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + bx_2 * q3 * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)
	  s2 = q4x2 * (2 * q2q4 - q1q3x2 - ax) + q1x2 * (2 * q1q2 + q3q4x2 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + bz_2 * q4 * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (bx_2 * q3 + bz_2 * q1) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + (bx_2 * q4 - bz_4 * q2) * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)
	  s3 = -q1x2 * (2 * q2q4 - q1q3x2 - ax) + q4x2 * (2 * q1q2 + q3q4x2 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-bx_4 * q3 - bz_2 * q1) * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (bx_2 * q2 + bz_2 * q4) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + (bx_2 * q1 - bz_4 * q3) * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)
	  s4 = q2x2 * (2 * q2q4 - q1q3x2 - ax) + q3x2 * (2 * q1q2 + q3q4x2 - ay) + (-bx_4 * q4 + bz_2 * q2) * (bx_2 * (0.5 - q3q3 - q4q4) + bz_2 * (q2q4 - q1q3) - mx) + (-bx_2 * q1 + bz_2 * q3) * (bx_2 * (q2q3 - q1q4) + bz_2 * (q1q2 + q3q4) - my) + bx_2 * q2 * (bx_2 * (q1q3 + q2q4) + bz_2 * (0.5 - q2q2 - q3q3) - mz)

	  # Normalize step magnitude
	  norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
	  s1 /= norm
	  s2 /= norm
	  s3 /= norm
	  s4 /= norm

	  # Compute rate of change of quaternion
	  qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
	  qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
	  qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
	  qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

	  # Integrate to yield quaternion
	  q1 += qDot1 * deltat
	  q2 += qDot2 * deltat
	  q3 += qDot3 * deltat
	  q4 += qDot4 * deltat

	  # Normalize quaternion
	  norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
	  self.q[0] = q1 / norm
	  self.q[1] = q2 / norm
	  self.q[2] = q3 / norm
	  self.q[3] = q4 / norm

	def attitude(self):
		# Get the data
		self.compFilter()
		self.calcHeading()

		# Print data
		print(" R: " + str(round(self.roll,1)) \
			+ " P: " + str(round(self.pitch,1)) \
			+ " Y: " + str(round(self.yaw,1)))

def main():
	# Set up class
	gyro = 500      # 250, 500, 1000, 2000 [deg/s]
	acc = 4         # 2, 4, 7, 16 [g]
	mag = 16        # 14, 16 [bit]
	tau = 0.98
	mpu = MPU(gyro, acc, mag, tau)

	# Calibrate the mag or provide values that have been verified with the visualizer
	# mpu.calibrateMagGuide()
	bias = [296.949, 330.334, -105.309]
	scale = [1.083, 0.949, 0.977]
	mpu.setMagCalibration(bias, scale)

	# Set up the sensors and calibrate the gyro with N points
	mpu.setUpIMU()
	mpu.setUpMAG()
	mpu.calibrateGyro(500)

	# Set timer
	lastUpdate = time.perf_counter()

	# Run until stopped
	try:
		while(True):
			# Get new values
			mpu.processValues()

			for ii in range(10):
				# Integration timer
				now = time.perf_counter()
				deltat = ((now - lastUpdate))
				lastUpdate = now

				# Run the sensor fusion
				mpu.madgwickFilter(mpu.ax, mpu.ay, mpu.az, mpu.gx, mpu.gy, mpu.gz, mpu.mx, mpu.my, mpu.mz, deltat)

	except KeyboardInterrupt:
		# End if user hits control c
		print("Closing")

# Main loop
if __name__ == '__main__':
	main()
