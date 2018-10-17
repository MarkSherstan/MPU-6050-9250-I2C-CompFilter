scaleFactorAccel = 8192; % 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
scaleFactorGyro = 65.5; % 250 deg/s --> 131, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4

a = arduino; %a = arduino('COM4','Uno','Libraries','I2C');
dev = i2cdev(a,'0x68');


% Activate the MPU 6050
writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int16');

% Configure the accelerometer - 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int16');

% Configure the gyroscope --> 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int16');


for i = 1:500
  [a g] = readMPU6050(dev);
end
