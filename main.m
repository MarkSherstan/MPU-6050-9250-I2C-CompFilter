function [] = main(ard,dev,totalTime,tau)
  close all

  % Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
  scaleFactorAccel = 8192;
  scaleFactorGyro = 65.5;

  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU 6050
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Find average offset for gyro
  %gyroCal = calibrateGyro(dev,scaleFactorGyro);
  gyroCal.x = -10.2107;
  gyroCal.y = 3.0496;
  gyroCal.z = 0.7554;

  % Set up figure, get properties, and label
  figure
  h1 = animatedline('Color',[1 0 0]);
  h2 = animatedline('Color',[0 1 0]);
  h3 = animatedline('Color',[0 0 1]);
  ax = gca;
  ax.YLim = [-90 90];
  xlabel('Time (s)')
  ylabel('Angle [deg]')
  legend('roll','pitch','yaw')

  % Initialize zero points
  pitch = 0;
  roll = 0;
  yaw = 0;
  previous = 0;

  % Start counters and timers
  i = 1;
  tic
  loopTimer(i) = toc;
  startTime = toc;

  % Loop for some amount of time
  while toc < totalTime
    % Read from MPU 6050
    [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro,gyroCal);

    % Find angles using complementary filter
    accelPitch = atan2(a.x, sqrt(a.y*a.y + a.z*a.z));
    accelRoll = atan2(a.y, sqrt(a.x*a.x + a.z*a.z));

    dt = toc - previous;

    pitch = (tau)*(pitch + g.x * dt) + (1 - tau)*(accelPitch);
    roll = (tau)*(roll + g.y * dt) + (1 - tau)*(accelRoll);
    yaw = (yaw + g.z * dt);

    % Update timers
    previous = toc;
    t = toc - startTime;

    % Plot the new points
    addpoints(h1,t,roll)
    addpoints(h2,t,pitch)
    addpoints(h3,t,yaw)

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
