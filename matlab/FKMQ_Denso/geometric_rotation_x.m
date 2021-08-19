function r_x = geometric_rotation_x(alpha)
%GEOMETRIC_ROTATION_Z Summary of this function goes here
%   Detailed explanation goes here
r_x = [1    0            0          0;
       0    cos(alpha)  -sin(alpha) 0;
       0    sin(alpha)   cos(alpha) 0;
       0    0            0          1];
end

