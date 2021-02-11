function x = trajectory(xm,xd,t)
%TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
x = xm*(xm.inv * xd)^t;
x = vec8(x);
x = x/norm(x);
x = DQ(x);
end

