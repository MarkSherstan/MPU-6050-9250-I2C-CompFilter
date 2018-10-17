function [a g] = readMPU6050(dev)

  % Read Accelerometer
  ACCEL_XOUT_H = readRegister(dev,hex2dec('3B'),'int8');
  ACCEL_XOUT_L = readRegister(dev,hex2dec('3C'),'int8');

  ACCEL_YOUT_H = readRegister(dev,hex2dec('3D'),'int8');
  ACCEL_YOUT_L = readRegister(dev,hex2dec('3E'),'int8');

  ACCEL_ZOUT_H = readRegister(dev,hex2dec('3F'),'int8');
  ACCEL_ZOUT_L = readRegister(dev,hex2dec('40'),'int8');

  a.x = typecast(int8([ACCEL_XOUT_H ACCEL_XOUT_L]),'int16');
  a.y = typecast(int8([ACCEL_YOUT_H ACCEL_YOUT_L]),'int16');
  a.z = typecast(int8([ACCEL_ZOUT_H ACCEL_ZOUT_L]),'int16');

  % Read Gyroscope
  GYRO_XOUT_H = readRegister(dev,hex2dec('43'),'int8');
  GYRO_XOUT_L = readRegister(dev,hex2dec('44'),'int8');

  GYRO_YOUT_H = readRegister(dev,hex2dec('45'),'int8');
  GYRO_YOUT_L = readRegister(dev,hex2dec('46'),'int8');

  GYRO_ZOUT_H = readRegister(dev,hex2dec('47'),'int8');
  GYRO_ZOUT_L = readRegister(dev,hex2dec('48'),'int8');

  g.x = typecast(int8([GYRO_XOUT_H GYRO_XOUT_L]),'int16');
  g.y = typecast(int8([GYRO_YOUT_H GYRO_YOUT_L]),'int16');
  g.z = typecast(int8([GYRO_ZOUT_H GYRO_ZOUT_L]),'int16');

end
