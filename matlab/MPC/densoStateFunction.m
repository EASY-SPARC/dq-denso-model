function [z] = densoStateFunction(x,u)
%DENSOSTATEFUNCTION Summary of this function goes here
%   Detailed explanation goes here
denso = DQ_DENSO;

z = zeros(14,1);

% disp(denso.jacobian(x(9:14)));
% disp(u);

quaternions = denso.jacobian(x(9:14))*u; % seria denso.fkm(theta);?
z(1) = quaternions(1);
z(2) = quaternions(2);
z(3) = quaternions(3);
z(4) = quaternions(4);
z(5) = quaternions(5);
z(6) = quaternions(6);
z(7) = quaternions(7);
z(8) = quaternions(8);
z(9) = u(1);
z(10) = u(2);
z(11) = u(3);
z(12) = u(4);
z(13) = u(5);
z(14) = u(6);

end

