function [A, Face] = NewCoords(roll, pitch, yaw)

% Set up the cube
initial = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1] - 0.5;
Face = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

% Create individual rotation matrices
yawMatrix   = [1  0           0;
                0  cosd(roll)  -sind(roll);
                0  sind(roll)   cosd(roll)];

pitchMatrix = [cosd(pitch)   0  sind(pitch);
                0             1  0;
                -sind(pitch)  0  cosd(pitch)];

rollMatrix  = [cosd(yaw)  -sind(yaw) 0;
                sind(yaw)  cosd(yaw)  0;
                0          0          1];

% Calculate the final rotation matrix
rotationMatrix = yawMatrix*pitchMatrix*rollMatrix;
A = initial * rotationMatrix;
