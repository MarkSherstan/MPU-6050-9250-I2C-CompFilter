clear all
close all
clc

% Connect to arduino
ard = arduino;
dev = i2cdev(ard,'0x68');

% Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
scaleFactorGyro = 65.5;

writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU 6050
writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

gyroCal = calibrateGyro(dev,scaleFactorGyro);
