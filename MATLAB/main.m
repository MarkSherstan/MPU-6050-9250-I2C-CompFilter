if (exist('a','var') != True
  % Connect to arduino
  ard = arduino;
  dev = i2cdev(ard, '0x68');

  % Set up and configure MPU for +/- 4g and 500 deg/s
  scaleFactorGyro = 65.5;

  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Calibrate the gyroscope
  gyroCal = calibrateGyro(dev,scaleFactorGyro);

  % Run the comp filter
  compFilter(ard, dev, gyroCal, tau)
else
  % If already started once and variables are saved just run the filter
  compFilter(ard, dev, gyroCal, tau)
end
