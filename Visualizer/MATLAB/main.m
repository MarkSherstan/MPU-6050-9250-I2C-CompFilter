% Clear up the workspace and visuals
clear all
close all

% Set up the class
gyro = 250;                       % 250, 500, 1000, 2000 [deg/s]
acc = 2;                          % 2, 4, 7, 16 [g]
tau = 0.98;                       % Time constant
port = '/dev/cu.usbmodem14101';   % Serial port name

vis = Visualizer(tau, acc, gyro, port);

% Open a serial port and calibrate the gyro
s = vis.openSerial();
vis.calibrateGyro(500, s);

% Configure the GUI
figureHandle = figure(1);
StopButton = uicontrol('Style','pushbutton','String','Stop & Close Serial Port','pos',[0, 0, 200, 25],'Callback','delete(gcbo)');
degreeLabelX = uicontrol('Style','text','String','X:  0 Degrees','pos',[450, 50, 100, 20],'parent',figureHandle);
degreeLabelY = uicontrol('Style','text','String','Y:  0 Degrees','pos',[450, 30, 100, 20],'parent',figureHandle);
degreeLabelZ = uicontrol('Style','text','String','Z:  0 Degrees','pos',[450, 10, 100, 20],'parent',figureHandle);
set(gcf,'Color','black');

% Loop until stopped by the user
while ishandle(StopButton)
	% Get the most up to date data
	vis.cubeGenerator(s);

	% Construct the cube
	view([1, 0, 0]);
	h = patch('Vertices',vis.cube,'Faces',vis.face,'FaceVertexCData',3,'FaceColor','flat');

	% Display the current angle
	set(degreeLabelX,'String', ['Roll:  '  num2str(round(vis.roll)) ' degrees']);
	set(degreeLabelY,'String', ['Pitch:  ' num2str(round(vis.pitch)) ' degrees']);
	set(degreeLabelZ,'String', ['Yaw:  '   num2str(round(vis.yaw)) ' degrees']);

	% More visual formatting
	axis off;
	axis([-1.1,1.1,-1.1,1.1,-1.1,1.1]);

	% Small delay for fresh data and for everything to process
	pause(0.001);

	% Update the graphic
	drawnow;
	delete(h);
end

% Close serial port and windows
vis.closeSerial(s)
