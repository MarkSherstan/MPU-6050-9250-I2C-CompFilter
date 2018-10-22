function [] = main(ard,dev)
  close all

  % Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
  scaleFactorAccel = 8192;
  scaleFactorGyro = 65.5;

  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU 6050
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Set up figure, get properties, and label
  figure
  h1 = animatedline;
  h2 = animatedline;
  h3 = animatedline;
  ax = gca;
  ax.YLim = [-4 4];
  xlabel('Time (s)')
  ylabel('Acceleration [g]')

  % Start a timer
  tic
  loopTimer = toc;
  startTime = toc;

  % Loop for some amount of time
  while toc < 15
    % Read from MPU 6050
    [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro);
    %fprintf('Accel x: %10.3f     Accel y: %10.3f     Accel z: %10.3f\n',a.x,a.y,a.z)

    t = toc - startTime;
    addpoints(h1,t,a.x)
    addpoints(h2,t,a.y)
    addpoints(h3,t,a.z)

    % Update axes
    ax.XLim = [0 t+10];
    drawnow

    % Wait for the timer to reach 250 Hz
    while ((toc - loopTimer) < 0.004); end
    loopTimer = toc;
  end

end
