% Clear the workspace
clear all

% Set up the class
gyro = 250;                       % 250, 500, 1000, 2000 [deg/s]
acc = 2;                          % 2, 4, 7, 16 [g]
tau = 0.98;                       % Time constant
port = '/dev/cu.usbmodem14101';   % Serial port name

mpu = MPU(tau, acc, gyro, port);

% Open a serial port and calibrate the gyro
s = mpu.openSerial();
mpu.calibrateGyro(100, s);

% Run for N point
for ii = 1:750
	% Get data and then display
	mpu.compFilter(s);
  fprintf('R: %0.1f  P: %0.1f  Y: %0.1f\n', mpu.roll, mpu.pitch, mpu.yaw)
end

% Close serial port
mpu.closeSerial(s)
