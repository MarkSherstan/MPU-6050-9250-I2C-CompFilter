clear all
close all
clc

% Connect to arduino
a = arduino;
dev = i2cdev(a,'0x68');

% Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
scaleFactorAccel = 8192;
scaleFactorGyro = 65.5;

writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int16'); % Activate MPU 6050
writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int16'); % Accelerometer
writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int16'); % Gyroscope

% Start a timer
tic
loopTimer = toc;

% Run a continous loop, get data, and eventually plot
while 1
  [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro);

  % Wait for the timer to reach 250 Hz
  while ((toc - loopTimer) < 0.004); end
  loopTimer = toc;
end
