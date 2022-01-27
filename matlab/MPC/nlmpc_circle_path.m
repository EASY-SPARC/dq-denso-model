function [x, y, z] = nlmpc_circle_path(center,r,ti,tf,Ts,thetai,thetaf)
%NLMPC_CIRCLE_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

steps = (tf - ti)/Ts;
x = center(1)*ones(steps+1, 1);
y = zeros(steps, 1);
z = zeros(steps, 1);
for k = 1:steps+1
    t = (k-1)*Ts + ti;
    theta = thetai*(1-(t-ti)/(tf-ti))+thetaf*(t-ti)/(tf-ti);
    y(k) = r*cos(theta) + center(2);
    z(k) = r*sin(theta) + center(3);
    
end