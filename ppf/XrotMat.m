function T = XrotMat(angle)
% rotation matrix from angle

T=[1 0 0 0;0 cos(angle) -sin(angle) 0;0 sin(angle) cos(angle) 0 ;0 0 0 1];

end