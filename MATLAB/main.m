function [] = main(ard,dev,gyroCal,tau)

  % Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
  scaleFactorAccel = 8192;
  scaleFactorGyro = 65.5;

  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU 6050
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Start timer and loop until stopped by user
  tic

  % Run for 30 seconds
  while toc < 30
    % Read from MPU 6050
    [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro,gyroCal);

    % Find angles using complementary filter
    accelPitch = atan2(a.x, sqrt(a.y*a.y + a.z*a.z));
    accelRoll = atan2(a.y, sqrt(a.x*a.x + a.z*a.z));

    dt = toc - previous;

    pitch = (tau)*(pitch + g.x * dt) + (1 - tau)*(accelPitch);
    roll = (tau)*(roll + g.y * dt) + (1 - tau)*(accelRoll);
    yaw = (yaw + g.z * dt);

    fprintf('Roll: %0.3f, Pitch: %0.3f, Yaw: %0.3f\n',roll, pitch, yaw)
  end

end
