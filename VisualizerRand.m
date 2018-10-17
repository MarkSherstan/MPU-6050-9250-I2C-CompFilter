clear all
close all

figureHandle = figure(1);
StopButton = uicontrol('Style','pushbutton','String','Stop & Close Serial Port','pos',[0, 0, 200, 25],'Callback','delete(gcbo)');
degreeLabelX = uicontrol('Style','text','String','X:  0 Degrees','pos',[450, 50, 100, 20],'parent',figureHandle);
degreeLabelY = uicontrol('Style','text','String','Y:  0 Degrees','pos',[450, 30, 100, 20],'parent',figureHandle);
degreeLabelZ = uicontrol('Style','text','String','Z:  0 Degrees','pos',[450, 10, 100, 20],'parent',figureHandle);
set(gcf,'Color','black');

x = 0;
y = 0;
z = 0;
i = 1;

while ishandle(StopButton)

	[vert, face] = NewCoords(x,y,z);
	view([1, 0, 0]);

	h = patch('Vertices',vert,'Faces',face,'FaceVertexCData',3,'FaceColor','flat');

	set(degreeLabelX,'String', ['X:  ' num2str(round(x)) ' degrees']);
	set(degreeLabelY,'String', ['Y:  ' num2str(round(y)) ' degrees']);
	set(degreeLabelZ,'String', ['Z:  ' num2str(round(z)) ' degrees']);

	axis off;
	axis([-1.1,1.1,-1.1,1.1,-1.1,1.1]);

i = i + 0.5;		

	x = i;
	x = updateAngle(x);
	% x = 0;

	y = i;
	y = updateAngle(y);
	% y = 0;

	z = i;
	z = updateAngle(z);
	% z = 0;

	pause(0.0001);

	drawnow;
	delete(h);

end

close all
