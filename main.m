function [] = main(ard,dev,totalTime,tau)
  close all

  % Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
  scaleFactorAccel = 8192;
  scaleFactorGyro = 65.5;

  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU 6050
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Find average offset for gyro
  gyroCal = calibrateGyro(dev,scaleFactorGyro);

  % Set up figure, get properties, and label
  figure
  h1 = animatedline('Color',[1 0 0]);
  h2 = animatedline('Color',[0 1 0]);
  h3 = animatedline('Color',[0 0 1]);
  ax = gca;
  ax.YLim = [-4 4];
  xlabel('Time (s)')
  ylabel('Angle [deg]')
  legend('theta_x','theta_y','theta_z')

  % Start counters and timers
  i = 1;
  tic
  loopTimer(i) = toc;
  startTime = toc;

  % Loop for some amount of time
  while toc < totalTime
    % Read from MPU 6050
    [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro,gyroCal);
    %fprintf('Accel x: %10.3f     Accel y: %10.3f     Accel z: %10.3f\n',a.x,a.y,a.z)

    accelPitch = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ));
    accelRoll = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ));

    dt = toc - previous;

    pitch = (tau)*(pitch + g.x * dt) + (1 - tau)*(accelPitch);
    roll = (tau)*(roll + g.y * dt) + (1 - tau)*(accelRoll);
    yaw = (yaw + g.z * dt); % Would this work?? --> 0.9*(yaw + g.z * dt) + 0.1*yaw

    previous = toc;


    t = toc - startTime;
    % If acceleration
    % addpoints(h1,t,a.x)
    % addpoints(h2,t,a.y)
    % addpoints(h3,t,a.z)

    % If gyro
    addpoints(h1,t,g.x)
    addpoints(h2,t,g.y)
    addpoints(h3,t,g.z)





    % Update axes
    if toc < 5
      ax.XLim = [0 t+5];
    else
      ax.XLim = [t-5 t+2];
    end

    drawnow

    % Wait for the timer to reach 250 Hz
    while ((toc - loopTimer(i)) < 0.004); end

    i = i + 1;
    loopTimer(i) = toc;
  end

  % Calculate sampling frequency
  for i = 2:length(loopTimer)-1
    freq(i) = loopTimer(i+1) - loopTimer(i);
  end
  fprintf('Average sample at %0.2f Hz\n',1/mean(freq))

end
