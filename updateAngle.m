function [angle] = updateAngle(angle)

% if (angle <= -360)
% 	angle = angle + 360;
% elseif (angle >= 360)
% 	angle = angle - 360;
% else
% 	return
% end

if (angle < 0)
	angle = angle + 360;
elseif (angle >= 360)
	angle = angle - 360;
else
	return
end

angle = updateAngle(angle);
