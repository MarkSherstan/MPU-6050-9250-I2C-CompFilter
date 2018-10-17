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



% READ DATA
ACCEL_XOUT_H = readRegister(dev,hex2dec('3B'),'int8')
ACCEL_XOUT_L = readRegister(dev,hex2dec('3C'),'int8')

% https://arduino.stackexchange.com/questions/36383/what-does-this-notation-stands-for-wire-read-8-wire-read
 % Page 7 --> https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
% https://www.mathworks.com/help/supportpkg/arduinoio/ref/shiftregisterread.html
% https://clickhouse-docs.readthedocs.io/en/latest/data_types/int_uint.html


% clear dev a
