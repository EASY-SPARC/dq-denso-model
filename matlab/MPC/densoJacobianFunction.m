function [A,Bmv] = densoJacobianFunction(x,u, Ts)
%DENSOJACOBIANFUNCTION Summary of this function goes here
%   Detailed explanation goes here
denso = DQ_DENSO;

A = zeros(14, 14);
Bmv = [denso.jacobian(x(9:14)); eye(6)];
end

