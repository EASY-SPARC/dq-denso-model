function r_z = geometric_rotation_z(theta)
%GEOMETRIC_ROTATION_Z Summary of this function goes here
%   Detailed explanation goes here
r_z = [ cos(theta) -sin(theta) 0 0;
        sin(theta)  cos(theta) 0 0;
        0           0          1 0;
        0           0          0 1];
end

