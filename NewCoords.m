function [A, Face] = NewCoords(roll, pitch, yaw)

Initial = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1]-0.5;
Face = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];

yaw_matrix   = [1  0           0;
                0  cosd(roll)  -sind(roll);
                0  sind(roll)   cosd(roll)];

pitch_matrix = [cosd(pitch)   0  sind(pitch);
                0             1  0;
                -sind(pitch)  0  cosd(pitch)];

roll_matrix  = [cosd(yaw)  -sind(yaw) 0;
                sind(yaw)  cosd(yaw)  0;
                0          0          1];

rot_matrix = yaw_matrix*pitch_matrix*roll_matrix;
A = Initial * rot_matrix;
