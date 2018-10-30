function [] = main(ard,dev,gyroCal,tau)

  close all

  % Set up and configure MPU 6050 for +/- 4g and 500 deg/s --> See README for more
  scaleFactorAccel = 8192;
  scaleFactorGyro = 65.5;

  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU 6050
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Initialize zero points
  roll = 0; pitch = 0; yaw = 0; previous = 0;

  % Set up visualizer and GUI
  figureHandle = figure(1);
  StopButton = uicontrol('Style','pushbutton','String','Stop & Close Serial Port','pos',[0, 0, 200, 25],'Callback','delete(gcbo)');
  degreeLabelX = uicontrol('Style','text','String','X:  0 Degrees','pos',[450, 50, 100, 20],'parent',figureHandle);
  degreeLabelY = uicontrol('Style','text','String','Y:  0 Degrees','pos',[450, 30, 100, 20],'parent',figureHandle);
  degreeLabelZ = uicontrol('Style','text','String','Z:  0 Degrees','pos',[450, 10, 100, 20],'parent',figureHandle);
  set(gcf,'Color','black');

  % Start timer and loop until stopped by user
  tic

  while ishandle(StopButton)
    % Read from MPU 6050
    [a g] = readMPU6050(dev,scaleFactorAccel,scaleFactorGyro,gyroCal);

    % Find angles using complementary filter
    accelPitch = atan2(a.x, sqrt(a.y*a.y + a.z*a.z));
    accelRoll = atan2(a.y, sqrt(a.x*a.x + a.z*a.z));

    dt = toc - previous;

    pitch = (tau)*(pitch + g.x * dt) + (1 - tau)*(accelPitch);
    roll = (tau)*(roll + g.y * dt) + (1 - tau)*(accelRoll);
    yaw = (yaw + g.z * dt);

    % Create the cube and update
  	[vert, face] = NewCoords(roll, pitch, yaw);
  	view([1, 0, 0]);

  	h = patch('Vertices',vert,'Faces',face,'FaceVertexCData',3,'FaceColor','flat');

  	set(degreeLabelX,'String', ['Roll:  ' num2str(round(roll)) ' degrees']);
  	set(degreeLabelY,'String', ['Pitch:  ' num2str(round(pitch)) ' degrees']);
  	set(degreeLabelZ,'String', ['Yaw:  ' num2str(round(yaw)) ' degrees']);

  	axis off;
  	axis([-1.1,1.1,-1.1,1.1,-1.1,1.1]);

  	roll = updateAngle(roll);
  	pitch = updateAngle(pitch);
  	yaw = updateAngle(yaw);

  	pause(0.0001);

  	drawnow;
  	delete(h);

  end

  close all

end
