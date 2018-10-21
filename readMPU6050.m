function [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro)

  % Read Accelerometer
  ACCEL_XOUT_H = readRegister(dev,hex2dec('3B'),'int8');
  ACCEL_XOUT_L = readRegister(dev,hex2dec('3C'),'int8');

  ACCEL_YOUT_H = readRegister(dev,hex2dec('3D'),'int8');
  ACCEL_YOUT_L = readRegister(dev,hex2dec('3E'),'int8');

  ACCEL_ZOUT_H = readRegister(dev,hex2dec('3F'),'int8');
  ACCEL_ZOUT_L = readRegister(dev,hex2dec('40'),'int8');

  a.x = double(typecast(int8([ACCEL_XOUT_H ACCEL_XOUT_L]),'int16')) / scaleFactorAccel;
  a.y = double(typecast(int8([ACCEL_YOUT_H ACCEL_YOUT_L]),'int16')) / scaleFactorAccel;
  a.z = double(typecast(int8([ACCEL_ZOUT_H ACCEL_ZOUT_L]),'int16')) / scaleFactorAccel;

  % Read Gyroscope
  GYRO_XOUT_H = readRegister(dev,hex2dec('43'),'int8');
  GYRO_XOUT_L = readRegister(dev,hex2dec('44'),'int8');

  GYRO_YOUT_H = readRegister(dev,hex2dec('45'),'int8');
  GYRO_YOUT_L = readRegister(dev,hex2dec('46'),'int8');

  GYRO_ZOUT_H = readRegister(dev,hex2dec('47'),'int8');
  GYRO_ZOUT_L = readRegister(dev,hex2dec('48'),'int8');

  g.x = double(typecast(int8([GYRO_XOUT_H GYRO_XOUT_L]),'int16')) / scaleFactorGyro;
  g.y = double(typecast(int8([GYRO_YOUT_H GYRO_YOUT_L]),'int16')) / scaleFactorGyro;
  g.z = double(typecast(int8([GYRO_ZOUT_H GYRO_ZOUT_L]),'int16')) / scaleFactorGyro;

end
